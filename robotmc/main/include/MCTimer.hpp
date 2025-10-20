#pragma once

#include "driver/mcpwm_timer.h"

class MCTimer
{
private:
    mcpwm_timer_handle_t timer_handle;
    int group_id;
    uint32_t resolution_hz;
    uint32_t pwm_freq_hz;

public:
    MCTimer(const int group_id, const uint32_t resolution_hz, const uint32_t pwm_freq_hz)
        : timer_handle(nullptr),
          group_id(group_id),
          resolution_hz(resolution_hz),
          pwm_freq_hz(pwm_freq_hz)
    {
        mcpwm_timer_config_t timer_config = {};

        timer_config.group_id = group_id;
        timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
        timer_config.resolution_hz = resolution_hz;
        timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
        timer_config.period_ticks = resolution_hz / pwm_freq_hz;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_handle));
    }

    void start() const
    {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer_handle));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_START_NO_STOP));
    }

    void stop() const
    {
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_STOP_EMPTY));
        ESP_ERROR_CHECK(mcpwm_timer_disable(timer_handle));
    }

    operator mcpwm_timer_handle_t() const
    {
        return timer_handle;
    }

    void cleanup()
    {
        if (timer_handle)
        {
            stop();
            ESP_ERROR_CHECK(mcpwm_del_timer(timer_handle));
            timer_handle = nullptr;
        }
    }
};