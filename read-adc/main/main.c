#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define ADC_PIN1 ADC_CHANNEL_6   //A2
#define ADC_PIN2 ADC_CHANNEL_3   //A3
#define ADC_UNIT ADC_UNIT_1      //ADC1
#define TAG "ADC"

void ADC_Task(void *pvParameters);

void app_main(void) {
    xTaskCreate(ADC_Task, "ADC_Task", 2048, NULL, 5, NULL);
}

void ADC_Task(void *pvParameters) {
    int adc_value1;
    int adc_value2;
    float adc_percent1;
    float adc_percent2;
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_PIN1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_PIN2, &config));

    for(;;) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_PIN1, &adc_value1));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_PIN2, &adc_value2));
        adc_percent1 = (float)adc_value1 / 4095 * 100;
        adc_percent2 = (float)adc_value2 / 4095 * 100;
        ESP_LOGI(TAG, "Potentio Value: %.2f%%", adc_percent1);
        ESP_LOGI(TAG, "Photo Value: %.2f%%\n", adc_percent2);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}