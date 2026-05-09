#pragma once

#include "freertos/FreeRTOS.h"

inline void taskDelayMillis(int dur_ms)
{
    vTaskDelay(pdMS_TO_TICKS(dur_ms));
}

inline uint16_t calculateDuration(float time_us, uint32_t resolution_hz)
{
    return uint16_t(time_us * resolution_hz / 1000000.0);
}
