#include "drill_platform.hpp"
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
// How far the drill platform is allowed to move down before hard-stop kicks in
constexpr float NUM_REV_HARD_STOP = -8; // TODO

StepperMotor drill_platform(pins::DRILL_PLATFORM_PINS.dir, pins::DRILL_PLATFORM_PINS.step,
                            pins::DRILL_PLATFORM_PINS.enable, STEPS_PER_REV);

bool drill_platform_hard_stop = false;
bool drill_hard_stop_overwrite = false;

bool drill_platform_limit = false;
bool drill_limit_overwrite = false;

bool handle_drill_hard_stop()
{
    static bool has_hard_stopped = false;

    if (drill_hard_stop_overwrite) {
        drill_platform_limit = false;
        has_hard_stopped = false;
        return false;
    }

    drill_platform_hard_stop = drill_platform.getNumRevolutions() < NUM_REV_HARD_STOP;

    if (drill_platform_hard_stop) {
        if (!has_hard_stopped) {
            has_hard_stopped = true;
            drill_platform.stop();
            DEBUG_LOG("Hard stopping drill platform.");
        }
    } else {
        has_hard_stopped = false;
    }

    return drill_platform_hard_stop;
}

// Handler for platform limit switches
// Currently just checks whether any platform switch is high, and if it is, stops the motors
// Returns whether any switches were high
bool handle_drill_limit_switches()
{
    static bool detect_pressed = false;

    if (drill_limit_overwrite) {
        drill_platform_limit = false;
        detect_pressed = false; // reset detection state
        return false;
    }

    // TODO: Option to disable. We don't want them stuck at comp!
    drill_platform_limit = static_cast<bool>(digitalRead(pins::DRILL_PLATFORM_LIMIT_SWITCH));

    // Only actually stop the motors the first time we detect switches are pressed down
    // we don't want to stop the motors from moving DOWN, only stop UPward movements.
    if (drill_platform_limit) {
        if (!detect_pressed) {
            detect_pressed = true;
            drill_platform.stop();
            return false;
        }
    } else if (detect_pressed) {
        detect_pressed = false;
    }

    return drill_platform_limit;
}

bool handle_drill_platform_limit_overwrite(bool mode)
{
    drill_limit_overwrite = mode;
    return drill_limit_overwrite;
}

bool handle_drill_platform_hard_stop_overwrite(bool mode)
{
    drill_hard_stop_overwrite = mode;
    return drill_hard_stop_overwrite;
}

} // namespace

// Moves the platform down 5 revolutions
void drill_platform_down()
{
    drill_platform.setEnabled(true);

    drill_platform.setDirection(StepperMotor::Direction::POSITIVE);

    drill_platform.start();
}

// Moves the platform up for 5 revolutions
void drill_platform_up()
{
    // Only allow upward movement when limit switches are not pressed
    if (!drill_platform_limit) {
        drill_platform.setEnabled(true);
        drill_platform.setDirection(StepperMotor::Direction::NEGATIVE);

        drill_platform.start();
    }
}

void drill_platform_stop()
{
    drill_platform.stop();
    drill_platform.setEnabled(false);
}

String set_drill_platform_limit_overwrite(const String& mode)
{
    if (static_cast<bool>(mode.equals("on"))) {
        DEBUG_LOG("Enabling lowering drill-platform limit overwrite.");
        handle_drill_platform_limit_overwrite(true);
    } else if (static_cast<bool>(mode.equals("off"))) {
        DEBUG_LOG("Disabling lowering drill-platform limit overwrite.");
        handle_drill_platform_limit_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
}

String set_drill_platform_hard_stop_overwrite(const String& mode)
{
    if (static_cast<bool>(mode.equals("on"))) {
        DEBUG_LOG("Enabling lowering drill-platform hard-stop overwrite.");
        handle_drill_platform_hard_stop_overwrite(true);
    } else if (static_cast<bool>(mode.equals("off"))) {
        DEBUG_LOG("Disabling lowering drill-platform hard-stop overwrite.");
        handle_drill_platform_hard_stop_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
}

void init_drill_platform()
{
    // Limit switches
    pinMode(pins::DRILL_PLATFORM_LIMIT_SWITCH, INPUT);
}

void update_drill_platform()
{
    handle_drill_limit_switches();
    handle_drill_hard_stop();
}

} // namespace cmd