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
//#include "protocol_examples_common.h"

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

#define Q1_OUTPUT 25
#define Q2_OUTPUT 26
#define Q3_OUTPUT 27
#define Q4_OUTPUT 14

// sentido = 0 --> girar derecha
// sentido = 1 --> girar izquierda
static void motor_girar(int sentido)
{
    // girar sentido horario (dcha): c0->c1->c2->c3->default
    if (sentido == 0)
    {
        for (int i = 0; i < 5; i++)
        {
            switch (i)
            {
            case 0:
                gpio_set_level(Q1_OUTPUT, 1);
                gpio_set_level(Q2_OUTPUT, 0);
                gpio_set_level(Q3_OUTPUT, 1);
                gpio_set_level(Q4_OUTPUT, 0);
                break;
            case 1:
                gpio_set_level(Q1_OUTPUT, 0);
                gpio_set_level(Q2_OUTPUT, 1);
                gpio_set_level(Q3_OUTPUT, 1);
                gpio_set_level(Q4_OUTPUT, 0);
                break;
            case 2:
                gpio_set_level(Q1_OUTPUT, 0);
                gpio_set_level(Q2_OUTPUT, 1);
                gpio_set_level(Q3_OUTPUT, 0);
                gpio_set_level(Q4_OUTPUT, 1);
                break;
            case 3:
                gpio_set_level(Q1_OUTPUT, 1);
                gpio_set_level(Q2_OUTPUT, 0);
                gpio_set_level(Q3_OUTPUT, 0);
                gpio_set_level(Q4_OUTPUT, 1);
                break;
            default:
                gpio_set_level(Q1_OUTPUT, 0);
                gpio_set_level(Q2_OUTPUT, 0);
                gpio_set_level(Q3_OUTPUT, 0);
                gpio_set_level(Q4_OUTPUT, 0);
                break;
            }
        }
    }
    // girar antihorario (izqda): c3->c2->c1->c0->default
    else if (sentido == 1)
    {
        for (int i = 0; i < 5; i++)
        {
            switch (i)
            {
            case 0:
                gpio_set_level(Q1_OUTPUT, 1);
                gpio_set_level(Q2_OUTPUT, 0);
                gpio_set_level(Q3_OUTPUT, 0);
                gpio_set_level(Q4_OUTPUT, 1);
                ESP_LOGI("MOTOR", "He hecho una iteracion");
                break;
            case 1:
                gpio_set_level(Q1_OUTPUT, 0);
                gpio_set_level(Q2_OUTPUT, 1);
                gpio_set_level(Q3_OUTPUT, 0);
                gpio_set_level(Q4_OUTPUT, 1);
                ESP_LOGI("MOTOR", "He hecho una iteracion");
                break;
            case 2:
                gpio_set_level(Q1_OUTPUT, 0);
                gpio_set_level(Q2_OUTPUT, 1);
                gpio_set_level(Q3_OUTPUT, 1);
                gpio_set_level(Q4_OUTPUT, 0);
                ESP_LOGI("MOTOR", "He hecho una iteracion");
                break;
            case 3:
                gpio_set_level(Q1_OUTPUT, 1);
                gpio_set_level(Q2_OUTPUT, 0);
                gpio_set_level(Q3_OUTPUT, 1);
                gpio_set_level(Q4_OUTPUT, 0);
                ESP_LOGI("MOTOR", "He hecho una iteracion");
                break;
            default:
                gpio_set_level(Q1_OUTPUT, 0);
                gpio_set_level(Q2_OUTPUT, 0);
                gpio_set_level(Q3_OUTPUT, 0);
                gpio_set_level(Q4_OUTPUT, 0);
                ESP_LOGI("MOTOR", "He hecho una iteracion");
                break;
            }
        }
    }
}