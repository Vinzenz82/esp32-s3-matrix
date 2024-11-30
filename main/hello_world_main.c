/* \copyright 2023 Zorxx Software. All rights reserved.
 * \license This file is released under the MIT License. See the LICENSE file for details.
 * \brief ESP32 Neopixel Driver Library Example Application
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "uthash.h"

#include "mqtt_client.h"

#include "font8x8_ib8x8u.h"

#include "driver/gpio.h"
#include "neopixel.h"

#define TAG "neopixel_test"
#define PIXEL_COUNT  64
#if defined(CONFIG_IDF_TARGET_ESP32S3)
   #define NEOPIXEL_PIN GPIO_NUM_14
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
   #define NEOPIXEL_PIN GPIO_NUM_8
#else
   #define NEOPIXEL_PIN GPIO_NUM_27
#endif

#if !defined(MAX)
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define CONFIG_BROKER_URL "mqtt://192.168.1.5"
esp_mqtt_client_handle_t client;
uint8_t mqtt_string_temperature[10] = {0};
uint8_t mqtt_string_temperature_length = 0;
uint8_t mqtt_string_special_char = 0x01;
uint8_t mqtt_string_message[255] = {0};
uint8_t mqtt_string_message_length = 0;
const uint8_t message_start_sign[1] = {0x10};

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

// Struktur für die Hash-Map-Einträge
typedef struct {
    char topic[64];                // Key: Topic-Name
    void (*callback)(const char *, const char *, int); // Value: Callback-Funktion
    UT_hash_handle hh;             // UTHash-Handle
} topic_handler_t;

// Globale Hash-Map
static topic_handler_t *handlers = NULL;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*--------------------- MQTT -----------------------------------*/
// Callback-Funktionen für verschiedene Topics
void handle_topic1(const char *topic, const char *data, int data_len) {
    printf("Handling topic1: %s, data: %.*s\n", topic, data_len, data);


    if( data_len <= 10 ) {
        strncpy((char*)mqtt_string_temperature, data, data_len);
        mqtt_string_temperature_length = (data_len + 1);
        mqtt_string_temperature[data_len] = mqtt_string_special_char; // add special symbol
    }
}

void handle_topic2(const char *topic, const char *data, int data_len) {
    printf("Handling topic2: %s, data: %.*s\n", topic, data_len, data);

    if(strncmp( data, "1F600", data_len) == 0) {
        mqtt_string_special_char = 1;
    }
    else if(strncmp( data, "1F641", data_len) == 0) { 
        mqtt_string_special_char = 224;
    }
    else if(strncmp( data, "02764", data_len) == 0) { 
        mqtt_string_special_char = 225;
    }
    else {
        mqtt_string_special_char = '?';
    }

    mqtt_string_temperature[mqtt_string_temperature_length-1] = mqtt_string_special_char;
}

void handle_topic3(const char *topic, const char *data, int data_len) {
    uint16_t len = (data_len < 255) ? data_len : 254;
    
    printf("Handling topic3: %s, data: %.*s\n", topic, data_len, data);


    memcpy(&mqtt_string_message, data, len);
    mqtt_string_message_length = len;
}

void handle_default(const char *topic, const char *data, int data_len) {
    printf("Unhandled topic: %s, data: %.*s\n", topic, data_len, data);
}

// Funktion, um eine neue Topic-Callback-Zuordnung hinzuzufügen
void add_topic_handler(const char *topic, void (*callback)(const char *, const char *, int)) {
    topic_handler_t *entry = (topic_handler_t *)malloc(sizeof(topic_handler_t));
    if (!entry) {
        printf("Failed to allocate memory for topic handler\n");
        return;
    }
    strncpy(entry->topic, topic, sizeof(entry->topic) - 1);
    entry->topic[sizeof(entry->topic) - 1] = '\0'; // Null-terminieren
    entry->callback = callback;
    HASH_ADD_STR(handlers, topic, entry); // Topic hinzufügen
}

// Funktion, um ein Topic zu suchen und das passende Callback aufzurufen
void dispatch_topic(const char *topic, int topic_len, const char *data, int data_len) {
    char topic_str[64];
    snprintf(topic_str, sizeof(topic_str), "%.*s", topic_len, topic); // Topic extrahieren

    topic_handler_t *entry = NULL;
    HASH_FIND_STR(handlers, topic_str, entry); // Topic in der Hash-Map suchen

    if (entry && entry->callback) {
        entry->callback(topic_str, data, data_len);
    } else {
        handle_default(topic_str, data, data_len); // Default-Handler
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        // Weiterleitung an das passende Callback
        dispatch_topic(event->topic, event->topic_len, event->data, event->data_len);

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .broker.address.port = 1883,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static bool drawChars(const uint8_t value[], const uint8_t count, const uint32_t rgb)
{
   tNeopixelContext neopixel = neopixel_Init(PIXEL_COUNT, NEOPIXEL_PIN);
   tNeopixel pixel_clear[PIXEL_COUNT] = {0};
   tNeopixel pixel[PIXEL_COUNT] = {0};
   const uint8_t* font_index = &font8x8_ib8x8u[value[0]][0];

   // check neopixel init state
   if(NULL == neopixel) {
      ESP_LOGE(TAG, "[%s] Initialization failed\n", __func__);
      return false;
   }

   // set index for each pixel
   for (int i = 0; i < PIXEL_COUNT; i++) {
      pixel[i].index = i; 
      //pixel[i].index = (i % 8) * 8 + (i / 8); // rotated by 90
      pixel_clear[i].index = i;
   }

   // loop through the chars
   for(uint8_t idx_char = 0; idx_char < count; idx_char++) {
      // set next char
      ESP_LOGI(TAG, "[%s] Choosing index %d", __func__, value[idx_char]);

      font_index = &font8x8_ib8x8u[value[idx_char]][0];

      // draw char into pixel buffer
      for(int r = 0; r < 8; r++) {
         for (int i = 0; i < 8; i++) {
            if((font_index[r] & (1 << i)) != 0 ) {
               pixel[(7-i)+(r*8)].rgb = rgb;
            }
         }
      }  

        // rotate by 180 deg
        for (int i = 0; i < 32; i++) {
            // Swap element at index i with its counterpart at index 63 - i
            uint32_t temp = pixel[i].rgb;
            pixel[i].rgb = pixel[63 - i].rgb;
            pixel[63 - i].rgb = temp;
        }

      // clear the screen if same char will be print
      if( (idx_char > 0) && ( value[idx_char - 1] == value[idx_char] ) ) {
        neopixel_SetPixel(neopixel, &pixel_clear[0], ARRAY_SIZE(pixel_clear));
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      // draw the char
      for(int i = 0; i < ARRAY_SIZE(pixel); ++i)
      {
         int index = (i % 8) * 8 + (i / 8);
         neopixel_SetPixel(neopixel, &pixel[index], 1); 
         vTaskDelay(pdMS_TO_TICKS(15));
      }
      vTaskDelay(pdMS_TO_TICKS(350));

      // clear the buffer
      // set index for each pixel
      for (int i = 0; i < PIXEL_COUNT; i++) {
         pixel[i].rgb = 0;
      }
   }

   neopixel_Deinit(neopixel);
   return true;
}

// Cleanup-Funktion zum Freigeben der Hash-Map
void cleanup_topic_handlers() {
    topic_handler_t *current, *tmp;
    HASH_ITER(hh, handlers, current, tmp) {
        HASH_DEL(handlers, current);
        free(current);
    }
}

void app_main(void)
{
   int msg_id;
   TickType_t start_tick, end_tick;
   uint32_t elapsed_time_ms;

   // set log levels
   esp_log_level_set("*", ESP_LOG_INFO);
   esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
   esp_log_level_set(TAG, ESP_LOG_VERBOSE);
   esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
   esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
   esp_log_level_set("transport", ESP_LOG_VERBOSE);
   esp_log_level_set("outbox", ESP_LOG_VERBOSE);

   //Initialize NVS
   esp_err_t ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK(ret);

   //Initialize WIFI
   ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
   wifi_init_sta();

   //start mqtt client
   add_topic_handler("/response/value", handle_topic1);
   add_topic_handler("/messages/1/1", handle_topic2);
   add_topic_handler("/messages/2/1", handle_topic3);

   mqtt_app_start();
   msg_id = esp_mqtt_client_subscribe(client, "/response/value", 1);
   ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
   msg_id = esp_mqtt_client_subscribe(client, "/messages/1/1", 1);
   ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
   msg_id = esp_mqtt_client_subscribe(client, "/messages/2/1", 1);
   ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

   for(;;)
   {
        start_tick = xTaskGetTickCount();

        // initiate a new update via MQTT
        msg_id = esp_mqtt_client_publish(client, "/request/value", "1", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        do{
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            if( mqtt_string_message_length > 0 ) {
                drawChars(&message_start_sign[0], 1, NP_RGB(0, 0, 20));
                drawChars(&mqtt_string_message[0], mqtt_string_message_length, NP_RGB(20, 2, 20));
            }
            else {
                drawChars(&mqtt_string_temperature[0], mqtt_string_temperature_length, NP_RGB(0, 25, 0)); 
            }
            
            end_tick = xTaskGetTickCount();
            elapsed_time_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
        } while( elapsed_time_ms < (59 * 1000));
        if(elapsed_time_ms < (60 * 1000)) {
            vTaskDelay(pdMS_TO_TICKS((60 * 1000) - elapsed_time_ms));
        }
   }

    // Cleanup-Logik (z. B. vor dem Beenden der Anwendung)
    cleanup_topic_handlers();
}
