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
// constexpr uint32_t STEPS_PER_REV = 200;

StepperMotor drill_platform(pins::DRILL_PLATFORM_PINS.dir, pins::DRILL_PLATFORM_PINS.step, 7);

bool drill_platform_limit = false;
bool drill_limit_overwrite = false;


// Handler for platform limit switches
// Currently just checks whether any platform switch is high, and if it is, stops the motors
// Returns whether any switches were high
bool handle_drill_limit_switches()
{
    static bool detect_pressed = false;

    if (drill_limit_overwrite) {
        detect_pressed = false;         // reset detection state
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

    return drill_platform_limit;
}

bool handle_drill_platform_overwrite(bool mode)
{
    if(mode) {
        drill_platform_limit = false;       // reset the platform limit to false to allow motor movements
    }
    drill_limit_overwrite = mode;
    return drill_limit_overwrite;
}

} // namespace

// Moves the platform down 5 revolutions
void drill_platform_up()
{
    // Only allow upward movement when limit switches are not pressed
    if (!drill_platform_limit) {
        drill_platform.setDirection(StepperMotor::Direction::POSITIVE);

        drill_platform.start();
    }
}

// Moves the platform up for 5 revolutions
void drill_platform_down()
{
    drill_platform.setDirection(StepperMotor::Direction::NEGATIVE);

    drill_platform.start();
}

void drill_platform_stop()
{
    drill_platform.stop();
}

String set_drill_platform_overwrite(const String& mode)
{
    if (mode.equals("on")) {
        DEBUG_LOG("Enabling lowering drill-platform limit overwrite.");
        handle_drill_platform_overwrite(true);
    } else if (mode.equals("off")) {
        DEBUG_LOG("Disabling lowering drill-platform limit overwrite.");
        handle_drill_platform_overwrite(false);
    } else {
        return CMD_ERR_MSG;
    }
    return {};
}

void init_drill_platform()
{
    // Limit switches
    pinMode(pins::DRILL_PLATFORM_LIMIT_SWITCH, INPUT);

    // lowering_platform_left.setSpeed(5);
    // lowering_platform_right.setSpeed(5);
    // drill_platform.setSpeed(5);
}

void update_drill_platform()
{
    handle_drill_limit_switches();

    // lowering_platform_left.update();
    // lowering_platform_right.update();
    // drill_platform.update();
}

} // namespace cmd