#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

const char *recursiveGuardTag = "recursiveMutexGuard";

class RecursiveMutexGuard
{
private:
    SemaphoreHandle_t mutex;
    const char *msg;
    bool ok;

    RecursiveMutexGuard(const RecursiveMutexGuard &) = delete;
    RecursiveMutexGuard &operator=(const RecursiveMutexGuard &) = delete;

public:
    RecursiveMutexGuard(SemaphoreHandle_t m, const char *s, const int timeout_ms = -1) : mutex(m), msg(s), ok(true)
    {
        if (m == NULL)
        {
            ok = false;
            ESP_LOGE(recursiveGuardTag, "Invalid mutex value: %s", msg);
            return;
        }
        const TickType_t timeout = timeout_ms < 0 ? portMAX_DELAY : timeout_ms / portTICK_PERIOD_MS;
        if (xSemaphoreTakeRecursive(mutex, timeout) != pdTRUE)
        {
            ok = false;
            ESP_LOGE(recursiveGuardTag, "Could not take mutex: %s", msg);
        }
    }
    operator bool() const
    {
        return ok;
    }
    ~RecursiveMutexGuard()
    {
        if (ok)
        {
            if (xSemaphoreGiveRecursive(mutex) != pdTRUE)
            {
                ESP_LOGE(recursiveGuardTag, "Could not give mutex: %s", msg);
            }
        }
    }
};
