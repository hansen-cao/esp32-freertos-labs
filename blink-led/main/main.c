#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#define led_pin 13
#define adc_pin ADC_CHANNEL_6
#define adc_unit ADC_UNIT_1
#define tag "ADC"

static QueueHandle_t queue;

void read_adc_task(void *pvParameters);
void blink_led_task(void *pvParameters);

void app_main(void) {
    xTaskCreate(read_adc_task, "read_adc_task", 2048, NULL, 1, NULL);
    xTaskCreate(blink_led_task, "blink_led_task", 2048, NULL, 1, NULL);
}

void read_adc_task(void *pvParameters) {
    int adc_value;
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = adc_unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_pin, &config));

    queue = xQueueCreate(10, sizeof(int));

    if(queue == NULL) {
        printf("Queue was not craeted\n");
    }

    for(;;) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adc_pin, &adc_value));
        ESP_LOGI(tag, "ADC Value: %d", adc_value);
        xQueueSend(queue, &adc_value, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void blink_led_task(void *pvParameters) {
    int adc_value;
    int delay_ms = 500;

    gpio_reset_pin(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);

    for(;;) {
        if(xQueueReceive(queue, &adc_value, 0) == pdPASS) {
            delay_ms = (float)adc_value / 4.095 + 1000;
        }

        gpio_set_level(led_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(led_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}