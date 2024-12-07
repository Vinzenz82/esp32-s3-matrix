// esp-idf includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// external lib includes
#include "SensorQMI8658.hpp"

#define TAG "QMI8658"

SensorQMI8658 qmi;

#ifndef SENSOR_SDA
#define SENSOR_SDA  11
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  12
#endif

#ifndef SENSOR_IRQ_1
#define SENSOR_IRQ_1  GPIO_NUM_10
#endif

#ifndef SENSOR_IRQ_2
#define SENSOR_IRQ_2  GPIO_NUM_13
#endif

#define GPIO_INPUT_PIN_SEL  ((1ULL<<SENSOR_IRQ_1) | (1ULL<<SENSOR_IRQ_2))

#define MASTER_FREQUENCY CONFIG_I2C_MASTER_FREQUENCY
#define PORT_NUMBER -1

/*static*/ QueueHandle_t gpio_evt_queue = NULL;

extern "C" void QMI8658_Update( void );

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// static void gpio_task_example(void* arg)
// {
//     uint32_t io_num;
//     for (;;) {
//         if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
//             printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
            
//         }
//     }
// }

void tapEventCallback()
{
    SensorQMI8658::TapEvent event = qmi.getTapStatus();
    switch (event) {
    case SensorQMI8658::SINGLE_TAP:
        ESP_LOGI(TAG, "Single-TAP");
        break;
    case SensorQMI8658::DOUBLE_TAP:
        ESP_LOGI(TAG, "Double-TAP");
        break;
    default:
        break;
    }
}

bool interruptFlag = false;

void setFlag(void)
{
    interruptFlag = true;
}

extern "C" void QMI8658_Init( void ) 
{
    int ret = DEV_WIRE_NONE;

    i2c_master_bus_config_t i2c_bus_config = {0};
        
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.i2c_port = PORT_NUMBER;
    i2c_bus_config.scl_io_num = GPIO_NUM_12;
    i2c_bus_config.sda_io_num = GPIO_NUM_11;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.intr_priority = 0;
    i2c_bus_config.flags.enable_internal_pullup = true;
    
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    qmi.setPins((int)SENSOR_IRQ_1);

    if (!qmi.begin(bus_handle, QMI8658_L_SLAVE_ADDRESS)) {
        ESP_LOGE(TAG, "qmi.begin failed");
    }

    ESP_LOGI(TAG, "Chip-ID: %x", qmi.getChipID());

        //** The recommended output data rate for detection is higher than 500HZ
    ret = qmi.configAccelerometer(
        /*
         * ACC_RANGE_2G
         * ACC_RANGE_4G
         * ACC_RANGE_8G
         * ACC_RANGE_16G
         * */
        SensorQMI8658::ACC_RANGE_2G,
        /*
         * ACC_ODR_1000H
         * ACC_ODR_500Hz
         * ACC_ODR_250Hz
         * ACC_ODR_125Hz
         * ACC_ODR_62_5Hz
         * ACC_ODR_31_25Hz
         * ACC_ODR_LOWPOWER_128Hz
         * ACC_ODR_LOWPOWER_21Hz
         * ACC_ODR_LOWPOWER_11Hz
         * ACC_ODR_LOWPOWER_3H
        * */
        SensorQMI8658::ACC_ODR_500Hz);
    ESP_LOGI(TAG, "configAccelerometer: %d", ret);

    // Enable the accelerometer
    qmi.enableAccelerometer();

    //* Priority definition between the x, y, z axes of acceleration.
    uint8_t priority = SensorQMI8658::PRIORITY5;  
    //* Defines the maximum duration (in sample) for a valid peak.
    //* In a valid peak, the linear acceleration should reach or be higher than the PeakMagThr
    //* and should return to quiet (no significant movement) within UDMThr, at the end of PeakWindow.
    uint8_t peakWindow = 20; //20 @500Hz ODR
    //* Defines the minimum quiet time before the second Tap happen.
    //* After the first Tap is detected, there should be no significant movement (defined by UDMThr) during the TapWindow.
    //* The valid second tap should be detected after TapWindow and before DTapWindow.
    uint16_t tapWindow = 50; //50 @500Hz ODR
    //* Defines the maximum time for a valid second Tap for Double Tap,
    //* count start from the first peak of the valid first Tap.
    uint16_t dTapWindow = 250; //250 @500Hz ODR
    //* Defines the ratio for calculating the average of the movement
    //* magnitude. The bigger of Gamma, the bigger weight of the latest  data.
    float alpha = 0.0625;
    //* Defines the ratio for calculating the average of the movement
    //* magnitude. The bigger of Gamma, the bigger weight of the latest data.
    float gamma = 0.25;
    //* Threshold for peak detection.
    float peakMagThr = 0.8; //0.8g square
    //* Undefined Motion threshold. This defines the threshold of the
    //* Linear Acceleration for quiet status.
    float UDMTh = 0.4; //0.4g square

    qmi.configTap(priority, peakWindow, tapWindow,
                  dTapWindow, alpha, gamma, peakMagThr, UDMTh);

    // Enable the Tap Detection and enable the interrupt
    qmi.enableTap(SensorQMI8658::INTERRUPT_PIN_1);

    // Set the Tap Detection callback function
    qmi.setTapEventCallBack(tapEventCallback);

    /*
     * set up INT pin and IRQ
     */

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    //xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(SENSOR_IRQ_1, gpio_isr_handler, (void*) SENSOR_IRQ_1);
    gpio_isr_handler_add(SENSOR_IRQ_2, gpio_isr_handler, (void*) SENSOR_IRQ_2);

}

extern "C" void QMI8658_Update( void ) 
{
    ESP_LOGI(TAG, "Status: %x", qmi.update());
    ESP_LOGI(TAG, "Temp: %f", qmi.getTemperature_C());
}