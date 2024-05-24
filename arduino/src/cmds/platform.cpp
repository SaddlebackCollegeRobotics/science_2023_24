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
// solve lowering from max height with limit switch on

StepperMotor lowering_platform_left(pins::PLATFORM_PINS[0].dir, pins::PLATFORM_PINS[0].step, 7);
StepperMotor lowering_platform_right(pins::PLATFORM_PINS[1].dir, pins::PLATFORM_PINS[1].step, 7);

bool left_platform_limit = false;
bool right_platform_limit = false;

bool platform_limit_overwrite = false;

// TODO: Better limit handling. Wrapper class?
// Handler for platform limit switches
// Currently just checks whether any platform switch is high, and if it is, stops the motors
// Returns whether any switches were high
bool handle_limit_switches()
{
    static bool detect_pressed = false;

    if (platform_limit_overwrite) {
        detect_pressed = false; // reset detection state
        return false;
    }

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

    return right_platform_limit || left_platform_limit;
}

bool handle_platform_overwrite(bool mode)
{
    if (mode) {
        left_platform_limit = false; // reset the platform limit to false to allow motor movements
        right_platform_limit = false;
    }
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
        DEBUG_LOG("Enabling lowering platform limit switches Overwrite.");
        handle_platform_overwrite(true);
    } else if (mode.equals("off")) {
        DEBUG_LOG("Disabling lowering platform limit switches Overwrite.");
        handle_platform_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
}

String read_platform_steps(const String& /*unused*/)
{
    return String((unsigned long)lowering_platform_left.getNumSteps()) + " + " + String((unsigned long)lowering_platform_right.getNumSteps());
}

void init_platform()
{
    // Limit switches
    pinMode(pins::PLATFORM_LIMIT_SWITCHES.left, INPUT);
    pinMode(pins::PLATFORM_LIMIT_SWITCHES.right, INPUT);
}

void update_platform()
{
    handle_limit_switches();
}

} // namespace cmd