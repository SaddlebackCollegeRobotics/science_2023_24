#pragma once

#include "pin_defines.hpp"
#include <Arduino.h>
#include <Servo.h>

// TODO: Rewrite to handle limit switches

// Simple class for the usongshine stepping motor (model #????)
class StepperMotor
{
public:
    enum class Direction
    {
        POSITIVE,
        NEGATIVE
    };

    StepperMotor(pins::pin_t dir_pin, pins::pin_t step_pin, pins::pin_t enable_pin, uint32_t step_period_millis = 2);

    void setDirection(Direction dir);
    void start();
    void stop();

private:
    pins::pin_t dir_pin_;
    pins::pin_t step_pin_;
    pins::pin_t enable_pin_;

    uint32_t step_freq_;
    Direction dir_ = Direction::POSITIVE;

    uint8_t id_;
    inline static uint8_t s_idx = 0;

    inline static bool s_setup_timer = false;
};