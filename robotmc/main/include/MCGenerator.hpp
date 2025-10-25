#pragma once

#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/gpio.h"

class MCGenerator
{
private:
    mcpwm_gen_handle_t gen_handle;
    gpio_num_t gpio_num;

public:
    MCGenerator(gpio_num_t gpio_num, mcpwm_oper_handle_t oper_handle, mcpwm_cmpr_handle_t cmpr_handle)
        : gen_handle(nullptr), gpio_num(gpio_num)
    {
        mcpwm_generator_config_t generator_config = {};
        generator_config.gen_gpio_num = gpio_num;
        ESP_ERROR_CHECK(mcpwm_new_generator(oper_handle, &generator_config, &gen_handle));

        esp_err_t e_timer = mcpwm_generator_set_action_on_timer_event(
            gen_handle,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        ESP_ERROR_CHECK(e_timer);

        esp_err_t e_compare = mcpwm_generator_set_action_on_compare_event(
            gen_handle,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_handle, MCPWM_GEN_ACTION_LOW));
        ESP_ERROR_CHECK(e_compare);
    }

    void pause(const bool high = false) const
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(gen_handle, high ? 1 : 0, true));
    }

    void resume() const
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(gen_handle, -1, true));
    }

    ~MCGenerator()
    {
        if (gen_handle)
        {
            ESP_ERROR_CHECK(mcpwm_del_generator(gen_handle));
            gen_handle = nullptr;
        }
    }
};