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
    int num_steps = 0;
    StepperMotor::Direction dir;
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
        
        if (stepper.dir == StepperMotor::Direction::POSITIVE) {
            stepper.num_steps++;
        } else {
            stepper.num_steps--;
        }

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
                           uint32_t steps_per_rev, uint32_t step_period_millis)
    : dir_pin_(dir_pin)
    , step_pin_(step_pin)
    , enable_pin_(enable_pin)
    , steps_per_rev_(steps_per_rev)
    , step_freq_(1000 / step_period_millis)
    , id_(s_idx++)
{
    steppers_info[id_] = {.enabled = false, .pin = step_pin, .num_steps = 0, .dir = Direction::POSITIVE};
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);

    setDirection(dir_);
    setEnabled(false);

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
    steppers_info[id_].dir = dir;

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

    DEBUG_LOG("Setting step pin %d to LOW", step_pin_);
    digitalWrite(step_pin_, LOW);
}

int StepperMotor::getNumSteps() const
{
    return steppers_info[id_].num_steps;
}

float StepperMotor::getNumRevolutions() const
{
    return static_cast<float>(steppers_info[id_].num_steps) / steps_per_rev_;
}

void StepperMotor::resetStepCount()
{
    steppers_info[id_].num_steps = 0;
}

void StepperMotor::setEnabled(bool is_enabled)
{
    DEBUG_LOG("Setting enable pin %d to HIGH", enable_pin_);

    digitalWrite(enable_pin_, is_enabled ? LOW : HIGH);
    steppers_info[id_].enabled = false;
}