#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "cJSON.h"

#include "./motor.h"

#define FOTORESISTOR1 ADC1_CHANNEL_4 // PIN_G32
#define FOTORESISTOR2 ADC1_CHANNEL_5 // PIN_G33
#define placa_solar ADC1_CHANNEL_7   //pin_G35
#define LED 22                       // PIN_G22
#define UMBRAL_ANALOGICO 3400
#define Q1_OUTPUT 25
#define Q2_OUTPUT 26
#define Q3_OUTPUT 27
#define Q4_OUTPUT 14

static const char *TAG = "MQTT_EXAMPLE";

static void mqtt_telemetry_send(int , int , int , esp_mqtt_client_handle_t , char[] , char[] , char[]);


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
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
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(esp_mqtt_client_handle_t client)
{
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void wifi_event_handler(void * event_handler_arg, esp_event_base_t event_base, int32_t event_id, void * event_data) {
  switch (event_id) {
  case WIFI_EVENT_STA_START:
    ESP_LOGI(TAG, "WiFi connecting ... \n");
    break;
  case WIFI_EVENT_STA_CONNECTED:
    ESP_LOGI(TAG, "WiFi connected ... \n");
    break;
  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGI(TAG, "WiFi lost connection ... \n");
    break;
  case IP_EVENT_STA_GOT_IP:
    ESP_LOGI(TAG, "WiFi got IP ... \n\n");
    break;
  default:
    break;
  }
}

void wifi_connection() {
  // 1 - Wi-Fi/LwIP Init Phase
  esp_netif_init(); // TCP/IP initiation      s1.1
  esp_event_loop_create_default(); // event loop                    s1.2
  esp_netif_create_default_wifi_sta(); // WiFi station                      s1.3
  wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init( & wifi_initiation); //                          s1.4
  // 2 - Wi-Fi Configuration Phase
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
  wifi_config_t wifi_configuration = {
    .sta = {
      .ssid = SSID,
      .password = PASS
    }
  };
  esp_wifi_set_config(ESP_IF_WIFI_STA, & wifi_configuration);
  // 3 - Wi-Fi Start Phase
  esp_wifi_start();
  // 4- Wi-Fi Connect Phase
  esp_wifi_connect();
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    gpio_set_direction(Q1_OUTPUT, GPIO_MODE_OUTPUT);
    gpio_set_direction(Q2_OUTPUT, GPIO_MODE_OUTPUT);
    gpio_set_direction(Q3_OUTPUT, GPIO_MODE_OUTPUT);
    gpio_set_direction(Q4_OUTPUT, GPIO_MODE_OUTPUT);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    wifi_connection();

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://demo.thingsboard.io",
        .event_handle = mqtt_event_handler,
        .port = 1883,
        .username = "4U9B75BMy4rz6zcC7Ozb",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    mqtt_app_start(client);

    gpio_set_direction(LED, GPIO_MODE_DEF_OUTPUT);

    printf("Valores LDR\n");
    printf("----------------------------------\n");

    uint32_t ldr_izqdo_val = 0, ldr_dcho_val = 0, placa_solar1 = 0;

    while (1)
    {
        ldr_izqdo_val = adc1_get_raw(FOTORESISTOR1);
        ldr_dcho_val = adc1_get_raw(FOTORESISTOR2);
        placa_solar1= adc1_get_raw(placa_solar);

        ESP_LOGI(TAG, "Placa móvil: %d", placa_solar1);
        ESP_LOGI(TAG, "LDR izquierdo: %d", ldr_izqdo_val);
        ESP_LOGI(TAG, "LDR derecho: %d", ldr_dcho_val);

          if (ldr_izqdo_val < UMBRAL_ANALOGICO || ldr_dcho_val < UMBRAL_ANALOGICO)
          {
              if (ldr_izqdo_val < ldr_dcho_val)
              {
                  while (ldr_izqdo_val < ldr_dcho_val * 0.985 || ldr_izqdo_val > ldr_dcho_val * 1.015)
                  {
                      motor_girar(1);
                  }
                  ESP_LOGI(TAG, "Girando a la izquierda...");
              }
              else
              {
                  while (ldr_dcho_val < ldr_izqdo_val * 0.985 || ldr_dcho_val > ldr_izqdo_val * 1.015)
                  {
                      motor_girar(0);
                  }
                  ESP_LOGI(TAG, "Girando a la derecha...");
              }
              vTaskDelay(pdMS_TO_TICKS(100));
          }
          else
          {
              vTaskDelay(pdMS_TO_TICKS(1000));
          }
        mqtt_telemetry_send(ldr_izqdo_val, ldr_dcho_val, placa_solar1, client, "ldr_izqdo", "ldr_dcho", "placa_movil");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void mqtt_telemetry_send(int ldr_val1, int ldr_val2, int placa_val, esp_mqtt_client_handle_t client, char ldr1[], char ldr2[], char placa1[])
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, ldr1, ldr_val1);
    cJSON_AddNumberToObject(root, ldr2, ldr_val2);
    cJSON_AddNumberToObject(root, placa1, placa_val);
    char *post_data = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0);
    cJSON_Delete(root);
    free(post_data);
}
