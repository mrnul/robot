#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

const char *mutexGuardTag = "mutexGuard";

class MutexGuard
{
private:
    SemaphoreHandle_t mutex;
    const char *msg;
    bool ok;

    MutexGuard(const MutexGuard &) = delete;
    MutexGuard &operator=(const MutexGuard &) = delete;

public:
    MutexGuard(SemaphoreHandle_t m, const char *s, const int timeout_ms = -1) : mutex(m), msg(s), ok(true)
    {
        if (m == NULL)
        {
            ok = false;
            ESP_LOGE(mutexGuardTag, "Invalid mutex value: %s", msg);
            return;
        }
        const TickType_t timeout = timeout_ms < 0 ? portMAX_DELAY : timeout_ms / portTICK_PERIOD_MS;
        if (xSemaphoreTake(mutex, timeout) != pdTRUE)
        {
            ok = false;
            ESP_LOGE(mutexGuardTag, "Could not take mutex: %s", msg);
        }
    }
    operator bool() const
    {
        return ok;
    }
    ~MutexGuard()
    {
        if (ok)
        {
            if (xSemaphoreGive(mutex) != pdTRUE)
            {
                ESP_LOGE(mutexGuardTag, "Could not give mutex: %s", msg);
            }
        }
    }
};
