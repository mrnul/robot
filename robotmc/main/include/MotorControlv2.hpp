#pragma once

#include "MCTimer.hpp"
#include "MCOperator.hpp"
#include "MCComparator.hpp"
#include "MCGenerator.hpp"
#include "DIO.hpp"

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
        DIO::reset(gpio_dir);
        DIO::setMode(gpio_dir, GPIO_MODE_OUTPUT);
        DIO::setLevel(gpio_dir, 0);
        set_value(0);
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
        DIO::reset(gpio_dir);
        DIO::setMode(gpio_dir, GPIO_MODE_OUTPUT);
        DIO::setLevel(gpio_dir, 0);
        set_value(0);
    }

    void set_value(const int value) const
    {
        cmpr.set_value(abs(value));
        DIO::setLevel(gpio_dir, value > 0);
        if(value == 0)
        {
            gen.pause();
        } else
        {
            gen.resume();
        }
    }
};
