#pragma once

#include "MCTimer.hpp"
#include "MCOperator.hpp"
#include "MCComparator.hpp"
#include "MCGenerator.hpp"
#include "driver/gpio.h"

class MotorControl
{
private:
    int group_id;
    gpio_num_t gpio_num1;
    gpio_num_t gpio_num2;
    uint32_t resolution_hz;
    uint32_t pwm_freq_hz;

    MCTimer timer;
    MCOperator oper;
    MCComparator cmpr;
    MCGenerator gen1;
    MCGenerator gen2;

public:
    MotorControl(const int group_id, const uint32_t resolution_hz, const uint32_t pwm_freq_hz, const gpio_num_t gpio_num1, const gpio_num_t gpio_num2)
        : group_id(group_id),
          gpio_num1(gpio_num1),
          gpio_num2(gpio_num2),
          resolution_hz(resolution_hz),
          pwm_freq_hz(pwm_freq_hz),
          timer(MCTimer(group_id, resolution_hz, pwm_freq_hz)),
          oper(MCOperator(group_id, timer)),
          cmpr(MCComparator(oper)),
          gen1(MCGenerator(gpio_num1, oper, cmpr)),
          gen2(MCGenerator(gpio_num2, oper, cmpr))
    {
        set_value(0);
        timer.start();
    }

    MotorControl(const int group_id, MCTimer &timer, const gpio_num_t gpio_num1, const gpio_num_t gpio_num2)
        : group_id(group_id),
          gpio_num1(gpio_num1),
          gpio_num2(gpio_num2),
          resolution_hz(0),
          pwm_freq_hz(0),
          timer(timer),
          oper(MCOperator(group_id, timer)),
          cmpr(MCComparator(oper)),
          gen1(MCGenerator(gpio_num1, oper, cmpr)),
          gen2(MCGenerator(gpio_num2, oper, cmpr))
    {
        set_value(0);
    }

    void set_value(const int value) const
    {
        cmpr.set_value(abs(value));
        if (value > 0)
        {
            gen2.pause();
            gen1.resume();
        }
        else if (value < 0)
        {
            gen1.pause();
            gen2.resume();
        }
        else
        {
            gen1.pause();
            gen2.pause();
        }
    }
};
