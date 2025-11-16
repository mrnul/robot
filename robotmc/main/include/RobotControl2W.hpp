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
    RobotControl2W(const uint32_t resolution_hz, const uint32_t pwm_freq_hz, const gpio_num_t gpio_r1, const gpio_num_t gpio_r2, const gpio_num_t gpio_l1, const gpio_num_t gpio_l2)
        : timer(MCTimer(0, resolution_hz, pwm_freq_hz)),
          right(MotorControl(0, timer, gpio_r1, gpio_r2)),
          left(MotorControl(0, timer, gpio_l1, gpio_l2)),
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

    ~RobotControl2W()
    {
        if (mutex)
        {
            vSemaphoreDelete(mutex);
            mutex = nullptr;
        }
    }
};