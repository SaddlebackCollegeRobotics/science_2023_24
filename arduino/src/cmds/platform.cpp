#include "platform.hpp"
#include "Stepper.h"
#include "StepperMotor/StepperMotor.hpp"
#include "WString.h"
#include "macros.hpp"
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

bool left_platform_limit = false;
bool right_platform_limit = false;
bool drill_platform_limit = false;

bool platform_limit_overwrite = false;
bool drill_limit_overwrite = false;

// Handler for platform limit switches
// Currently just checks whether any platform switch is high, and if it is, stops the motors
// Returns whether any switches were high
bool handle_limit_switches()
{
    if (platform_limit_overwrite) {
        return false;
    }

    static bool detect_pressed = false;
    // TODO: Option to disable. We don't want them stuck at comp!
    left_platform_limit = static_cast<bool>(digitalRead(pins::PLATFORM_LIMIT_SWITCHES.left));
    right_platform_limit = static_cast<bool>(digitalRead(pins::PLATFORM_LIMIT_SWITCHES.right));

    // Only actually stop the motors the first time we detect switches are pressed down
    // we don't want to stop the motors from moving DOWN, only stop UPward movements.
    if (left_platform_limit || right_platform_limit) {
        if (!detect_pressed) {
            detect_pressed = true;
            lowering_platform_left.stop();
            lowering_platform_right.stop();
            return false;
        }
    } else if (detect_pressed) {
        detect_pressed = false;
    }

    //!!! Old implementation
    // if (left_platform_limit || right_platform_limit) {
    //     // Stop platform motors
    //     lowering_platform_left.stop();
    //     lowering_platform_right.stop();
    //     return false;
    // }

    // bool drill_switch_val = static_cast<bool>(digitalRead(pins::DRILL_PLATFORM_LIMIT_SWITCH));

    // if (drill_switch_val) {
    //     drill_platform.stop();
    //     return false;
    // }

    return right_platform_limit || left_platform_limit;
}

bool handle_platform_overwrite(bool mode)
{
    platform_limit_overwrite = mode;
    return platform_limit_overwrite;
}

} // namespace

// Moves the platform down 5 revolutions
void platform_up()
{
    // Only allow upward movement when limit switches are not pressed
    if (!left_platform_limit && !right_platform_limit) {
        lowering_platform_left.setDirection(StepperMotor::Direction::POSITIVE);
        lowering_platform_right.setDirection(StepperMotor::Direction::POSITIVE);

        lowering_platform_left.start();
        lowering_platform_right.start();
    }
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

String set_platform_overwrite(const String& mode)
{
    if (mode.equals("on")) {
        DEBUG_LOG("Enabling lowering platform limit switches.");
        handle_platform_overwrite(true);
    } else if (mode.equals("off")) {
        DEBUG_LOG("Disabling lowering platform limit switches.");
        handle_platform_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
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