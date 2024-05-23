#include "platform.hpp"
#include "Stepper.h"
#include "StepperMotor/StepperMotor.hpp"
#include "WString.h"
#include "pin_defines.hpp"
#include "pins_arduino.h"
#include <Arduino.h>

namespace cmd {

namespace {

// TODO
// constexpr uint32_t STEPS_PER_REV = 200;

StepperMotor lowering_platform_left(pins::PLATFORM_PINS[0].dir, pins::PLATFORM_PINS[0].step, 7);
StepperMotor lowering_platform_right(pins::PLATFORM_PINS[1].dir, pins::PLATFORM_PINS[1].step, 7);
StepperMotor drill_platform(pins::DRILL_PLATFORM_PINS.dir, pins::DRILL_PLATFORM_PINS.step, 7);

// Handler for platform limit switches
// Currently just checks whether any platform switch is high, and if it is, stops the motors
// Returns whether any switches were high
bool handle_limit_switches()
{
    // TODO: Option to disable. We don't want them stuck at comp!
    bool left_switch_val = static_cast<bool>(digitalRead(pins::PLATFORM_LIMIT_SWITCHES.left));
    bool right_switch_val = static_cast<bool>(digitalRead(pins::PLATFORM_LIMIT_SWITCHES.right));
    if (left_switch_val || right_switch_val) {
        // Stop platform motors
        lowering_platform_left.stop();
        lowering_platform_right.stop();
        return false;
    }

    bool drill_switch_val = static_cast<bool>(digitalRead(pins::DRILL_PLATFORM_LIMIT_SWITCH));

    if (drill_switch_val) {
        drill_platform.stop();
        return false;
    }

    return false;
}

} // namespace

// Moves the platform down 5 revolutions
void platform_up()
{
    lowering_platform_left.setDirection(StepperMotor::Direction::POSITIVE);
    lowering_platform_right.setDirection(StepperMotor::Direction::POSITIVE);

    lowering_platform_left.start();
    lowering_platform_right.start();
}

// Moves the platform up for 5 revolutions
void platform_down()
{
    lowering_platform_left.setDirection(StepperMotor::Direction::NEGATIVE);
    lowering_platform_right.setDirection(StepperMotor::Direction::NEGATIVE);

    lowering_platform_left.start();
    lowering_platform_right.start();
}

void platform_stop()
{
    lowering_platform_left.stop();
    lowering_platform_right.stop();
}

void init_platform()
{
    // Limit switches
    pinMode(pins::DRILL_PLATFORM_LIMIT_SWITCH, INPUT);
    pinMode(pins::PLATFORM_LIMIT_SWITCHES.left, INPUT);
    pinMode(pins::PLATFORM_LIMIT_SWITCHES.right, INPUT);

    // lowering_platform_left.setSpeed(5);
    // lowering_platform_right.setSpeed(5);
    // drill_platform.setSpeed(5);
}

void update_platform()
{
    handle_limit_switches();

    // lowering_platform_left.update();
    // lowering_platform_right.update();
    // drill_platform.update();
}

} // namespace cmd