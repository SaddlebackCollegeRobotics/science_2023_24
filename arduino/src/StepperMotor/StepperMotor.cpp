#include "StepperMotor.hpp"

#include "Arduino.h"
#include "pin_defines.hpp"
#include "task_queue.hpp"
#include <TimerOne.h>

namespace {

constexpr auto NUM_STEPPERS = 4;

struct stepper_info
{
    bool enabled = false;
    pins::pin_t pin = 0;
};

util::array<stepper_info, NUM_STEPPERS> steppers_info{};

bool stepper_pwm_callback(void*)
{
    for (auto& stepper : steppers_info) {
        // Pin 0 denotes uninitialized stepper
        if (stepper.pin == 0) {
            continue;
        }

        if (!stepper.enabled) {
            continue;
        }

        // DEBUG_LOG("Stepping motor %d (state = %s)", stepper.pin, stepper.state ? "HIGH" : "LOW");

        digitalWrite(stepper.pin, HIGH);
        delayMicroseconds(200);
        digitalWrite(stepper.pin, LOW);
        delayMicroseconds(200);
        digitalWrite(stepper.pin, HIGH);
    }

    return true;
}

} // namespace

StepperMotor::StepperMotor(pins::pin_t dir_pin, pins::pin_t step_pin, pins::pin_t enable_pin,
                           uint32_t step_period_millis)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , enable_pin_(enable_pin)
    , step_freq_(1000 / step_period_millis)
    , id_(s_idx++)
{
    steppers_info[id_] = {.enabled = false, .pin = step_pin};
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);

    setDirection(dir_);

    stop();

    DEBUG_ASSERT(id_ < NUM_STEPPERS, "Too many steppers instantiated! (%d > %d)", id_, NUM_STEPPERS - 1);
    if (!s_setup_timer) {
        s_setup_timer = true;
        // Assume Timer1 is already initialized
        task_timer.every(2, stepper_pwm_callback);
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
    digitalWrite(enable_pin_, LOW);
}

void StepperMotor::stop()
{
    steppers_info[id_].enabled = false;
    digitalWrite(enable_pin_, HIGH);
}