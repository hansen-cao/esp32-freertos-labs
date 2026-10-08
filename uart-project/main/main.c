#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define led_pin 13
#define adc_pin ADC_CHANNEL_6
#define adc_unit ADC_UNIT_1

static volatile int adc_msg;
static volatile int delay;
static volatile int msg_flag = 0;
static char *msg_ptr = NULL;

static const uart_port_t uart_num = UART_NUM_0;
static SemaphoreHandle_t adc_empty;
static SemaphoreHandle_t adc_full;

void serial_task(void *pvParameters);
void print_task(void *pvParameters);
void adc_task(void *pvParameters);
void led_task(void *pvParameters);

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1028, 0, 0, NULL, 0));

    adc_empty = xSemaphoreCreateBinary();
    adc_full = xSemaphoreCreateBinary();

    xTaskCreate(serial_task, "serial_task", 4096, NULL, 1, NULL);
    xTaskCreate(print_task, "print_task", 4096, NULL, 1, NULL);
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 1, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 1, NULL);
}

void serial_task(void *pvParameters) {
    char buffer[128] = {0};
    uint8_t data;
    int index = 0;

    for(;;) {
        int length = uart_read_bytes(uart_num, &data, 1, portMAX_DELAY);

        if(length > 0) {
                if(data != '\r') {
                    buffer[index] = (char) data;
                    index++;
                } else if(index > sizeof(buffer)) {
                    uart_write_bytes(uart_num, "Buffer out of memory!\n", 22);
                    index = 0;
                } else {
                    msg_ptr = pvPortMalloc(index + 1);

                    if(msg_ptr == NULL) {
                        uart_write_bytes(uart_num, "Memory allocation failed!\n", 26);
                    } else {
                        buffer[index] = '\0';
                        strcpy(msg_ptr, buffer);
                        msg_flag = 1;
                        index = 0;
                        length = 0;
                    }
                }
            }
        }
    }

void print_task(void *pvParameters) {
    char buffer[32] = {0};

    for(;;) {
        if(msg_flag == 1) {
            if(strcmp(msg_ptr, "pot") == 0) {
                xSemaphoreGive(adc_empty);

                if(xSemaphoreTake(adc_full, portMAX_DELAY) == pdTRUE) {
                    int len = snprintf(buffer, sizeof(buffer), "ADC value: %d\n", adc_msg);
                    uart_write_bytes(uart_num, buffer, len);
                    // Delay between led blinks is 1 - 2 seconds
                    delay = (float) adc_msg / 4.095 + 1000;
                }

                vTaskDelay(pdMS_TO_TICKS(500));
            } else {
                uart_write_bytes(uart_num, msg_ptr, strlen(msg_ptr));
                uart_write_bytes(uart_num, "\n", 1);
            }

            vPortFree(msg_ptr);
            msg_flag = 0;
            uart_flush_input(uart_num);
        } else {
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
}

void adc_task(void *pvParameters) {
    int adc_value;
    adc_oneshot_unit_handle_t adc_handle;

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

    for(;;) {
        if(xSemaphoreTake(adc_empty, portMAX_DELAY) == pdTRUE) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adc_pin, &adc_value));
            adc_msg = adc_value;
            xSemaphoreGive(adc_full);
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void led_task(void *pvParameters) {
    gpio_reset_pin(led_pin);
    gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);

    for(;;) {
        gpio_set_level(led_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(delay));
        gpio_set_level(led_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(delay));
    }
}
