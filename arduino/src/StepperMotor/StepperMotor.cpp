#include "StepperMotor.hpp"

#include "Arduino.h"
#include "pin_defines.hpp"
#include <arduino-timer.h>

StepperMotor::StepperMotor(pins::pin_t dir_pin, pins::pin_t step_pin, uint32_t step_period_millis)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , step_period_millis_(step_period_millis)
{
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);

    setDirection(dir_);
}

void StepperMotor::setDirection(Direction dir)
{
    dir_ = dir;

    switch (dir) {
    case Direction::POSITIVE:
        digitalWrite(dir_pin_, HIGH);
        return;
    case Direction::NEGATIVE:
        digitalWrite(dir_pin_, LOW);
        return;
    }
}

// Sets direfction
void StepperMotor::setRunState(bool state)
{
    is_running_ = state;
}

void StepperMotor::update()
{
    if (!is_running_) {
        return;
    }

    if (millis() - last_update_millis_ >= step_period_millis_) {
        DEBUG_LOG("Stepping motor (%d)", static_cast<int>(last_write_mode_));
        last_update_millis_ = millis();

        last_write_mode_ = !last_write_mode_;
        digitalWrite(step_pin_, static_cast<uint8_t>(last_write_mode_));
    }
}