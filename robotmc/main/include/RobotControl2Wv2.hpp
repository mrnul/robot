#pragma once

#include <mutex>

#include "MotorControlv2.hpp"
#include "MCTimer.hpp"
#include "driver/gpio.h"

using std::mutex;

class RobotControl2Wv2
{
private:
    MCTimer timer;
    MotorControlv2 right;
    MotorControlv2 left;
    mutex m;

public:
    RobotControl2Wv2(const uint32_t resolution_hz, const uint32_t pwm_freq_hz, const gpio_num_t dir_r, const gpio_num_t pwm_r, const gpio_num_t dir_l, const gpio_num_t pwm_l)
        : timer(MCTimer(0, resolution_hz, pwm_freq_hz)),
          right(MotorControlv2(0, timer, dir_r, pwm_r)),
          left(MotorControlv2(0, timer, dir_l, pwm_l))
    {
        timer.start();
    }

    void setVr(const int vr)
    {
        std::lock_guard<mutex> lock(m);
        right.setValue(vr);
    }

    void setVl(const int vl)
    {
        std::lock_guard<mutex> lock(m);
        left.setValue(vl);
    }

    void setZero()
    {
        std::lock_guard<mutex> lock(m);
        right.setValue(0);
        left.setValue(0);
    }
};