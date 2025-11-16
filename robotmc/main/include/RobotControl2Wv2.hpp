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

    void setVr(const int vr) const
    {
        MutexGuard lock(mutex, "RobotControl2W::set_vr");
        right.setValue(vr);
    }

    void setVl(const int vl) const
    {
        MutexGuard lock(mutex, "RobotControl2W::set_vl");
        left.setValue(vl);
    }

    void setZero() const
    {
        MutexGuard lock(mutex, "RobotControl2W::set_zero");
        right.setValue(0);
        left.setValue(0);
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