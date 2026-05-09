#pragma once

#include "MCTimer.hpp"
#include "MCOperator.hpp"
#include "MCComparator.hpp"
#include "MCGenerator.hpp"

class MotorControlv2
{
private:
    int group_id;
    gpio_num_t gpio_dir;
    gpio_num_t gpio_pwm;
    uint32_t resolution_hz;
    uint32_t pwm_freq_hz;

    MCTimer timer;
    MCOperator oper;
    MCComparator cmpr;
    MCGenerator gen;

public:
    MotorControlv2(const int group_id, const uint32_t resolution_hz, const uint32_t pwm_freq_hz, const gpio_num_t dir, const gpio_num_t pwm)
        : group_id(group_id),
          gpio_dir(dir),
          gpio_pwm(pwm),
          resolution_hz(resolution_hz),
          pwm_freq_hz(pwm_freq_hz),
          timer(MCTimer(group_id, resolution_hz, pwm_freq_hz)),
          oper(MCOperator(group_id, timer)),
          cmpr(MCComparator(oper)),
          gen(MCGenerator(pwm, oper, cmpr))
    {
        gpio_reset_pin(gpio_dir);
        gpio_set_direction(gpio_dir, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_dir, 0);
        setValue(0);
        timer.start();
    }

    MotorControlv2(const int group_id, MCTimer &timer, const gpio_num_t dir, const gpio_num_t pwm)
        : group_id(group_id),
          gpio_dir(dir),
          gpio_pwm(pwm),
          timer(timer),
          oper(MCOperator(group_id, timer)),
          cmpr(MCComparator(oper)),
          gen(MCGenerator(pwm, oper, cmpr))
    {
        gpio_reset_pin(gpio_dir);
        gpio_set_direction(gpio_dir, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_dir, 0);
        setValue(0);
    }

    void setValue(const int value) const
    {
        cmpr.setValue(abs(value));
        gpio_set_level(gpio_dir, value > 0);
        if(value == 0)
        {
            gen.pause();
        } else
        {
            gen.resume();
        }
    }
};
