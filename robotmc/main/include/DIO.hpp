#pragma once

#include "driver/gpio.h"

class DIO
{
public:
    static bool reset(gpio_num_t gpio_num)
    {
        return gpio_reset_pin(gpio_num) == ESP_OK;
    }

    static bool setMode(gpio_num_t gpio_num, gpio_mode_t mode)
    {
        return gpio_set_direction(gpio_num, mode) == ESP_OK;
    }

    static bool setLevel(gpio_num_t gpio_num, uint32_t level)
    {
        return gpio_set_level(gpio_num, level) == ESP_OK;
    }

    static bool setPullMode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
    {
        return gpio_set_pull_mode(gpio_num, pull) == ESP_OK;
    }

    static bool getLevel(gpio_num_t gpio_num)
    {
        return gpio_get_level(gpio_num) == 1;
    }
};