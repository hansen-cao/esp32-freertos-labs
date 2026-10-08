#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static char *msg_ptr = NULL;
static volatile uint8_t msg_ready = 0;

void read_serial_task(void *pvParameters);
void print_msg_task(void *pvParameters);

void app_main(void) {
    xTaskCreate(read_serial_task, "read_serial_task", 2048, NULL, 1, NULL);
    xTaskCreate(print_msg_task, "print_msg_task", 2048, NULL, 1, NULL);
}

void read_serial_task(void *pvParameters) {
    int index = 0;
    char buffer[255] = {0};

    for(;;) {
        int c = getchar();
        
        if(msg_ready == 1) {
            index = 0;
            memset(buffer, 0, sizeof(buffer));
        }

        if(c == -1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            if(index > 254) {
                index = 0;
                memset(buffer, 0, sizeof(buffer));
                printf("Message too long\n");
            } else if(c != '\n') {
                buffer[index] = (char)c;
                index++;
            } else {
                if(msg_ready == 0) {
                    msg_ptr = pvPortMalloc(index + 1);
                }
                
                if(msg_ptr == NULL) {
                    printf("Memory allocation failed\n");
                } else {
                    strcpy(msg_ptr, buffer);
                    msg_ready = 1;
                }
            }
        }
    }
}

void print_msg_task(void *pvParameters) {
    for(;;) {
        if(msg_ready == 1 && msg_ptr != NULL) {
            printf("\nHigh water mark (bytes): %d\n", uxTaskGetStackHighWaterMark(NULL));
            printf("Heap after malloc (bytes): %d\n", xPortGetFreeHeapSize());

            printf("%s\n", msg_ptr);
            vPortFree(msg_ptr);
            msg_ptr = NULL;
            msg_ready = 0;
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}