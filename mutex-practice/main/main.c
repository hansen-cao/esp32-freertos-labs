#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"

void task_A(void *pvParameters);
void task_B(void *pvParameters);
void task_C(void *pvParameters);

#define adc_pin ADC_CHANNEL_6
#define adc_unit ADC_UNIT_1

static adc_oneshot_unit_handle_t adc_handle;
static SemaphoreHandle_t mutex_handle;

void app_main(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_pin, &config));

    mutex_handle = xSemaphoreCreateMutex();

    if(mutex_handle == NULL) {
        printf("Mutex was not created\n");
    }

    xTaskCreate(task_A, "task_A", 4096, NULL, 1, NULL);
    xTaskCreate(task_B, "task_B", 4096, NULL, 1, NULL);
    xTaskCreate(task_C, "task_C", 4096, NULL, 1, NULL);
}

void task_A(void *pvParameters) {
    if(xSemaphoreTake(mutex_handle, portMAX_DELAY) == pdTRUE) {
        int adc_value;

        for(;;) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adc_pin, &adc_value));
            xSemaphoreGive(mutex_handle);

            printf("ADC Value: %d\n", adc_value);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_B(void *pvParameters) {
    if(xSemaphoreTake(mutex_handle, portMAX_DELAY) == pdTRUE) {
        int adc_value;

        for(;;) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adc_pin, &adc_value));
            xSemaphoreGive(mutex_handle);

            printf("ADC Value: %d\n", adc_value);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_C(void *pvParameters) {
    if(xSemaphoreTake(mutex_handle, portMAX_DELAY) == pdTRUE) {
        int adc_value;

        for(;;) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adc_pin, &adc_value));
            xSemaphoreGive(mutex_handle);

            printf("ADC Value: %d\n", adc_value);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}