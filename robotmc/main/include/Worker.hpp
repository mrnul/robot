#pragma once

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

template <class T>
using WorkerCallbackType = bool (*)(T &, void *);

template <class T>
class Worker
{
private:
    QueueHandle_t queue;
    WorkerCallbackType<T> callback;
    SemaphoreHandle_t doneSemaphore;
    void *extraData;

private:
    static void taskEntryPoint(void *param)
    {
        Worker<T> *self = (Worker<T> *)(param);
        T item;
        
        ESP_LOGI(pcTaskGetName(NULL), "Worker task started");
        while (true)
        {
            if (xQueueReceive(self->queue, &item, portMAX_DELAY) == pdTRUE)
            {
                if (!self->callback(item, self->extraData))
                    break;
            }
            else
            {
                break;
            }
        }
        ESP_LOGI(pcTaskGetName(NULL), "Worker task stopped");
        xSemaphoreGive(self->doneSemaphore);
        vTaskDelete(NULL);
    }

public:
    Worker(WorkerCallbackType<T> callback, void *extraData = nullptr, const int queueLen = 10)
        : queue(xQueueCreate(queueLen, sizeof(T))),
          callback(callback),
          doneSemaphore(xSemaphoreCreateBinary()),
          extraData(extraData)
    {
        if (queue == NULL)
        {
            ESP_LOGE("Worker", "Could not create queue");
        }
        if (doneSemaphore == NULL)
        {
            ESP_LOGE("Worker", "Could not create semaphore");
        }
    }

    void setExtraData(void *data)
    {
        extraData = data;
    }

    bool start(const char *name, const uint32_t stackDepth = 4096, const UBaseType_t priority = tskIDLE_PRIORITY + 1)
    {
        return xTaskCreate(taskEntryPoint, name, stackDepth, this, priority, NULL) == pdTRUE;
    }

    bool submit(const T &item)
    {
        return xQueueSendToBack(queue, &item, 0) == pdTRUE;
    }

    void wait()
    {
        xSemaphoreTake(doneSemaphore, portMAX_DELAY);
    }

    ~Worker()
    {
        if (queue != NULL)
        {
            vQueueDelete(queue);
            queue = NULL;
        }
        if (doneSemaphore != NULL)
        {
            vSemaphoreDelete(doneSemaphore);
            doneSemaphore = NULL;
        }
    }
};