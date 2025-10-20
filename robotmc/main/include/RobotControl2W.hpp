#pragma once

#include "MotorControl.hpp"
#include "MCTimer.hpp"
#include "MutexGuard.hpp"

class RobotControl2W
{
private:
    MCTimer timer;
    MotorControl right;
    MotorControl left;
    SemaphoreHandle_t mutex;

public:
    RobotControl2W(const uint32_t resolution_hz, const uint32_t pwm_freq_hz, const int gpio_r1, const int gpio_r2, const int gpio_l1, const int gpio_l2)
        : timer(MCTimer(0, resolution_hz, pwm_freq_hz)),
          right(MotorControl(0, timer, gpio_r1, gpio_r2)),
          left(MotorControl(0, timer, gpio_l1, gpio_l2)),
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

    ~RobotControl2W()
    {
        if (mutex)
        {
            vSemaphoreDelete(mutex);
            mutex = nullptr;
        }
    }
};