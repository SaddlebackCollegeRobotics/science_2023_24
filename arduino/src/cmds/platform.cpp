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
constexpr uint32_t STEPS_PER_REV = 200;
// solve lowering from max height with limit switch on

constexpr float NUM_REV_HARD_STOP = -14; // TODO

StepperMotor lowering_platform_left(pins::PLATFORM_PINS[0].dir, pins::PLATFORM_PINS[0].step,
                                    pins::PLATFORM_PINS[0].enable, STEPS_PER_REV);
StepperMotor lowering_platform_right(pins::PLATFORM_PINS[1].dir, pins::PLATFORM_PINS[1].step,
                                     pins::PLATFORM_PINS[1].enable, STEPS_PER_REV);

bool left_platform_hard_stop = false;
bool right_platform_hard_stop = false;
bool platform_hard_stop_overwrite = false;

bool left_platform_limit = false;
bool right_platform_limit = false;
bool platform_limit_overwrite = false;

bool handle_hard_stop()
{
    static bool left_has_hard_stopped = false;
    static bool right_has_hard_stopped = false;

    if (platform_hard_stop_overwrite) {
        left_platform_limit = false;    // reset detection state
        right_platform_limit = false;   // reset detection state
        left_has_hard_stopped = false;  // reset on-press handler
        right_has_hard_stopped = false; // reset on-press handler
        return false;
    }

    left_platform_hard_stop = lowering_platform_left.getNumRevolutions() < NUM_REV_HARD_STOP;
    right_platform_hard_stop = lowering_platform_right.getNumRevolutions() < NUM_REV_HARD_STOP;

    if (left_platform_hard_stop) {
        if (!left_has_hard_stopped) {
            left_has_hard_stopped = true;
            lowering_platform_left.stop();
        }
    } else if (left_has_hard_stopped) {
        left_has_hard_stopped = false;
    }

    if (right_platform_hard_stop) {
        if (!right_has_hard_stopped) {
            right_has_hard_stopped = true;
            lowering_platform_right.stop();
        }
    } else if (right_has_hard_stopped) {
        right_has_hard_stopped = false;
    }

    return left_platform_hard_stop || right_platform_hard_stop;
}

// TODO: Better limit handling. Wrapper class?
// Handler for platform limit switches
// Currently just checks whether any platform switch is high, and if it is, stops the motors
// Returns whether any switches were high
bool handle_limit_switches()
{
    static bool detect_left_pressed = false;
    static bool detect_right_pressed = false;

    if (platform_limit_overwrite) {
        left_platform_limit = false;  // reset detection state
        right_platform_limit = false; // reset detection state
        detect_left_pressed = false;  // reset on-press handler
        detect_right_pressed = false; // reset on-press handler
        return false;
    }

    left_platform_limit = static_cast<bool>(digitalRead(pins::PLATFORM_LIMIT_SWITCHES.left));
    right_platform_limit = static_cast<bool>(digitalRead(pins::PLATFORM_LIMIT_SWITCHES.right));

    // Only actually stop the motors the first time we detect switches are pressed down
    // we don't want to stop the motors from moving DOWN, only stop UPward movements.
    if (left_platform_limit) {
        if (!detect_left_pressed) {
            // Reset the step count after reaching the top
            lowering_platform_left.resetStepCount();

            detect_left_pressed = true;
            lowering_platform_left.stop();
            DEBUG_LOG("Left platform motor limit");
        }
    } else if (detect_left_pressed) {
        detect_left_pressed = false;
        DEBUG_LOG("Reset left platform motor limit");
    }

    if (right_platform_limit) {
        if (!detect_right_pressed) {
            // Reset the step count after reaching the top
            lowering_platform_right.resetStepCount();

            detect_right_pressed = true;
            lowering_platform_right.stop();
            DEBUG_LOG("Right platform motor limit");
        }
    } else if (detect_right_pressed) {
        detect_right_pressed = false;
        DEBUG_LOG("Reset right platform motor limit");
    }

    return right_platform_limit || left_platform_limit;
}

bool handle_platform_limit_overwrite(bool mode)
{
    platform_limit_overwrite = mode;
    return platform_limit_overwrite;
}

bool handle_platform_hard_stop_overwrite(bool mode)
{
    platform_hard_stop_overwrite = mode;
    return platform_hard_stop_overwrite;
}

} // namespace

// Moves the platform down 5 revolutions
void platform_up()
{
    DEBUG_LOG("Limits: (left = %d, right = %d)", (int)left_platform_limit, (int)right_platform_limit);
    // Only allow each motor to move upwards when their respective limit is not pressed
    if (!left_platform_limit) {
        lowering_platform_left.setDirection(StepperMotor::Direction::POSITIVE);
        DEBUG_LOG("Platform moving left motor");
        lowering_platform_left.start();
    }

    if (!right_platform_limit) {
        lowering_platform_right.setDirection(StepperMotor::Direction::POSITIVE);
        DEBUG_LOG("Platform moving right motor");
        lowering_platform_right.start();
    }
}

// Moves the platform up for 5 revolutions
void platform_down()
{
    if (!left_platform_hard_stop) {
        lowering_platform_left.setDirection(StepperMotor::Direction::NEGATIVE);
        lowering_platform_left.start();
    }

    if (!right_platform_hard_stop) {
        lowering_platform_right.setDirection(StepperMotor::Direction::NEGATIVE);
        lowering_platform_right.start();
    }
}

void platform_stop()
{
    lowering_platform_left.stop();
    lowering_platform_right.stop();
}

String platform_set_enabled(const String& is_enabled)
{
    if (static_cast<bool>(is_enabled.equals("true"))) {
        lowering_platform_left.setEnabled(true);
        lowering_platform_right.setEnabled(true);
    } else if (static_cast<bool>(is_enabled.equals("false"))) {
        lowering_platform_left.setEnabled(false);
        lowering_platform_right.setEnabled(false);
    }

    return {};
}

String set_platform_limit_overwrite(const String& mode)
{
    if (static_cast<bool>(mode.equals("off"))) {
        DEBUG_LOG("Enabling lowering platform limit switches Overwrite.");
        handle_platform_limit_overwrite(true);
    } else if (static_cast<bool>(mode.equals("on"))) {
        DEBUG_LOG("Disabling lowering platform limit switches Overwrite.");
        handle_platform_limit_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
}

String set_platform_hard_stop_overwrite(const String& mode)
{
    if (static_cast<bool>(mode.equals("off"))) {
        DEBUG_LOG("Enabling lowering platform limit switches Overwrite.");
        handle_platform_hard_stop_overwrite(true);
    } else if (static_cast<bool>(mode.equals("on"))) {
        DEBUG_LOG("Disabling lowering platform limit switches Overwrite.");
        handle_platform_hard_stop_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
}

String read_platform_revs(const String& /*unused*/)
{
    return String(lowering_platform_left.getNumRevolutions());
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
    handle_hard_stop();
}

} // namespace cmd