#include "StepperMotor.hpp"

#include "Arduino.h"
#include "pin_defines.hpp"
#include <TimerOne.h>

namespace {

constexpr auto NUM_STEPPERS = 4;

struct stepper_info
{
    bool enabled = false;
    bool state = LOW;
    pins::pin_t pin = 0;
};

util::array<stepper_info, NUM_STEPPERS> steppers_info{};

void stepper_pwn_callback()
{

    for (auto& stepper : steppers_info) {
        // Pin 0 denotes uninitialized stepper
        if (stepper.pin == 0) {
            continue;
        }

        if (!stepper.enabled) {
            continue;
        }

        DEBUG_LOG("Stepping motor %d (state = %s)", stepper.pin, stepper.state ? "HIGH" : "LOW");

        digitalWrite(stepper.pin, stepper.state ? HIGH : LOW);
        stepper.state = !stepper.state;
    }
}

} // namespace

StepperMotor::StepperMotor(pins::pin_t dir_pin, pins::pin_t step_pin, uint32_t step_period_millis)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , step_period_millis_(step_period_millis)
    , id_(s_idx++)
{
    steppers_info[id_] = {.enabled = false, .state = LOW, .pin = step_pin};
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);

    setDirection(dir_);

    DEBUG_ASSERT(id_ < NUM_STEPPERS, "Too many steppers instantiated! (%d > %d)", id_, NUM_STEPPERS - 1);
    if (!s_setup_interrupt) {
        s_setup_interrupt = true;
        // Assume Timer1 is already initialized
        Timer1.attachInterrupt(stepper_pwn_callback);
    }
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

void StepperMotor::start()
{
    steppers_info[id_].enabled = true;
}

void StepperMotor::stop()
{
    steppers_info[id_].enabled = false;
}