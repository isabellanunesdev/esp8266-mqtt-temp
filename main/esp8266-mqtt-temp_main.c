#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "mqtt_client.h"

#include "vars_config.h"

#define DHT11_PIN GPIO_NUM_4

const int WIFI_CONNECTED_BIT = BIT0;

typedef struct {
    uint8_t temperature;
    uint8_t humidity;
} dht_data_t;

static EventGroupHandle_t wifi_event_group;
static QueueHandle_t dht_queue;

static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        printf("Conectado ao Wi-Fi\n");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        printf("Desconectado do Wi-Fi\n");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init() {
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .bssid_set = false,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
}

void DHT11_init(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_pullup_en(pin);
}

void DHT11_start_signal(gpio_num_t pin) {
    gpio_set_level(pin, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    gpio_set_level(pin, 1);
    ets_delay_us(30);
}

int DHT11_check_response(gpio_num_t pin) {
    ets_delay_us(40);
    if (gpio_get_level(pin) == 0) {
        ets_delay_us(80);
        if (gpio_get_level(pin) == 1) {
            ets_delay_us(80);
            return 1;
        }
    }
    return 0;
}

uint8_t DHT11_read_byte(gpio_num_t pin) {
    uint8_t i, byte = 0;
    for (i = 0; i < 8; i++) {
        uint32_t timeout = 100000;
        while (gpio_get_level(pin) == 0) {
            if (--timeout == 0) {
                printf("Timeout esperando pelo nível alto\n");
                return 0;
            }
        }

        ets_delay_us(40);

        if (gpio_get_level(pin) == 1) {
            byte |= (1 << (7 - i));
        }

        timeout = 100000;

        while (gpio_get_level(pin) == 1) {
            if (--timeout == 0) {
                printf("Timeout esperando pelo nível baixo\n");
                return 0;
            }
        }
    }
    return byte;
}

void DHT11_read(gpio_num_t pin, uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0};
    DHT11_start_signal(pin);
    if (DHT11_check_response(pin)) {
        data[0] = DHT11_read_byte(pin);
        data[1] = DHT11_read_byte(pin);
        data[2] = DHT11_read_byte(pin);
        data[3] = DHT11_read_byte(pin);
        data[4] = DHT11_read_byte(pin);

        uint8_t checksum = data[0] + data[1] + data[2] + data[3];
        if (data[4] == checksum) {
            *humidity = data[0];
            *temperature = data[2];
        }
        else {
            printf("Erro de checksum ao ler o DHT11\n");
        }
    }
    else {
        printf("Sem resposta do DHT11\n");
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        printf("Conectado ao broker MQTT\n");
        break;
    case MQTT_EVENT_DISCONNECTED:
        printf("Desconectado do broker MQTT\n");
        vTaskDelay(30000 / portTICK_PERIOD_MS);
        esp_mqtt_client_reconnect(client);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    mqtt_event_handler_cb(event_data);
}

void dht_task(void *pvParameter) {
    dht_data_t data;

    while (1)
    {
        DHT11_read(DHT11_PIN, &data.temperature, &data.humidity);

        printf("Temperatura: %d°C, Umidade: %d%%\n", data.temperature, data.humidity);

        xQueueSend(dht_queue, &data, portMAX_DELAY);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void mqtt_task(void *pvParameter) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_BROKER_URI,
        .port = 1883,
        .disable_auto_reconnect = true,
    };

    esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    dht_data_t data;

    while (1) {
        if (xQueueReceive(dht_queue, &data, portMAX_DELAY)) {
            char temp_str[5];
            char hum_str[5];
            snprintf(temp_str, sizeof(temp_str), "%d", data.temperature);
            snprintf(hum_str, sizeof(hum_str), "%d", data.humidity);

            esp_mqtt_client_publish(mqtt_client, MQTT_TEMPERATURE_TOPIC, temp_str, 0, 1, 0);
            esp_mqtt_client_publish(mqtt_client, MQTT_HUMIDITY_TOPIC, hum_str, 0, 1, 0);
        }
    }
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();
    DHT11_init(DHT11_PIN);
    dht_queue = xQueueCreate(5, sizeof(dht_data_t));

    xTaskCreate(&dht_task, "dht_task", 2048, NULL, 10, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);
}
