#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

void scanner_task(void *pvParameters);
void uploader_task(void *pvParameters);

static SemaphoreHandle_t slotEmpty;
static SemaphoreHandle_t slotFull;

typedef struct {
    int id;
    int weight;
    int time;
    bool valid;
} Package;

static Package package;

void app_main(void) {
    srand(time(NULL));

    slotEmpty = xSemaphoreCreateBinary();
    slotFull = xSemaphoreCreateBinary();
    xSemaphoreGive(slotEmpty);

    xTaskCreate(scanner_task, "scanner_task", 4096, NULL, 1, NULL);
    xTaskCreate(uploader_task, "uploader_task", 4096, NULL, 1, NULL);
}

void scanner_task(void *pvParameters) {
    package.id = 0;

    for(;;) {
        if(xSemaphoreTake(slotEmpty, portMAX_DELAY) == pdTRUE) {
            package.id++;
            package.weight = (rand() % 30 ) + 1;
            package.time = (rand() % 7) + 1;
            package.valid = true;
            xSemaphoreGive(slotFull);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void uploader_task(void *pvParameters) {
    for(;;) {
        if(xSemaphoreTake(slotFull, portMAX_DELAY) == pdTRUE) {
            if(package.valid) {
                int id = package.id;
                int weight = package.weight;
                int time = package.time;
                package.valid = false;

                printf("\nPackage received:\n");
                printf("id: %d\n", id);
                printf("weight: %d\n", weight);
                printf("time: %d\n", time);

                xSemaphoreGive(slotEmpty);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}