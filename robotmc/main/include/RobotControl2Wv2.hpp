#pragma once

#include "MotorControlv2.hpp"
#include "MCTimer.hpp"
#include "MutexGuard.hpp"
#include "driver/gpio.h"

class RobotControl2Wv2
{
private:
    MCTimer timer;
    MotorControlv2 right;
    MotorControlv2 left;
    SemaphoreHandle_t mutex;

public:
    RobotControl2Wv2(const uint32_t resolution_hz, const uint32_t pwm_freq_hz, const gpio_num_t dir_r, const gpio_num_t pwm_r, const gpio_num_t dir_l, const gpio_num_t pwm_l)
        : timer(MCTimer(0, resolution_hz, pwm_freq_hz)),
          right(MotorControlv2(0, timer, dir_r, pwm_r)),
          left(MotorControlv2(0, timer, dir_l, pwm_l)),
          mutex(xSemaphoreCreateMutex())
    {
        timer.start();
    }

    void set_vr(const int vr) const
    {
        MutexGuard lock(mutex, "RobotControl2W::set_vr");
        right.set_value(vr);
    }

    void set_vl(const int vl) const
    {
        MutexGuard lock(mutex, "RobotControl2W::set_vl");
        left.set_value(vl);
    }

    ~RobotControl2Wv2()
    {
        if (mutex)
        {
            vSemaphoreDelete(mutex);
            mutex = nullptr;
        }
    }
};