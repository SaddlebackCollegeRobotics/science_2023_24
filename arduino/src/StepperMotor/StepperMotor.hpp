#pragma once

#include "pin_defines.hpp"
#include <arduino-timer.h>

// Simple class for the usongshine stepping motor (model #????)
class StepperMotor
{
public:
    enum class Direction
    {
        POSITIVE,
        NEGATIVE
    };

    StepperMotor(pins::pin_t dir_pin, pins::pin_t step_pin, uint32_t step_period_millis = 10);

    void setStepPeriod(uint32_t step_period_millis) { step_period_millis_ = step_period_millis; }
    void setDirection(Direction dir);
    void setRunState(bool state);

    void update();

private:
    pins::pin_t dir_pin_;
    pins::pin_t step_pin_;

    uint32_t step_period_millis_ = 10;
    Direction dir_ = Direction::POSITIVE;

    unsigned long last_update_millis_ = 0;
    bool last_write_mode_ = LOW;
    bool is_running_ = false;
};