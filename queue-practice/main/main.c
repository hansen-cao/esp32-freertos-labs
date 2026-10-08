#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void add_queue_task(void *pvParameters);
void read_queue_task(void *pvParameters);

static QueueHandle_t struct_queue;

typedef struct {
    int counter;
    bool isOdd;
} queue_message;

void app_main(void) {
    struct_queue = xQueueCreate(5, sizeof(queue_message));

    if(struct_queue == NULL) {
        printf("Queue was not created\n");
        return;
    }

    xTaskCreate(add_queue_task, "add_queue_task", 2048, NULL, 1, NULL);
    xTaskCreate(read_queue_task, "read_queue_task", 2048, NULL, 1, NULL);
}

void add_queue_task(void *pvParameters) {
    queue_message message_to_send = {0, false};

    for(;;) {
        xQueueSend(struct_queue, &message_to_send, 0);
        message_to_send.counter++;
        message_to_send.isOdd = (message_to_send.counter % 2 != 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void read_queue_task(void *pvParameteres) {
    queue_message received_message;

    for(;;) {
        if(xQueueReceive(struct_queue, &received_message, portMAX_DELAY) == pdPASS) {
            printf("Counter: %d\n", received_message.counter);
            printf("Odd or Even: %s\n", received_message.isOdd ? "Odd" : "Even");
        }
    }
}