#pragma once

#include "freertos/FreeRTOS.h"

void delay(int dur_ms)
{
    vTaskDelay(dur_ms / portTICK_PERIOD_MS);
}

uint16_t calculate_duration(float time_us, uint32_t resolution_hz)
{
    return uint16_t(time_us * resolution_hz / 1000000.0);
}
