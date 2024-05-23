#pragma once

#include "pin_defines.hpp"
#include <Arduino.h>

// Simple class for the usongshine stepping motor (model #????)
class StepperMotor
{
public:
    enum class Direction
    {
        POSITIVE,
        NEGATIVE
    };

    StepperMotor(pins::pin_t dir_pin, pins::pin_t step_pin, uint32_t step_period_millis = 2);

    void setStepPeriod(uint32_t step_period_millis) { step_period_millis_ = step_period_millis; }
    void setDirection(Direction dir);
    void start();
    void stop();

    void update();

private:
    pins::pin_t dir_pin_;
    pins::pin_t step_pin_;

    uint32_t step_period_millis_ = 10;
    Direction dir_ = Direction::POSITIVE;

    uint8_t id_;

    inline static bool s_setup_interrupt = false;
    inline static uint8_t s_idx = 0;
};