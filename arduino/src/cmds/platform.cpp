#include "platform.hpp"
#include "Arduino.h"
#include "StepperMotor/StepperMotor.hpp"
#include "WString.h"
#include "pin_defines.hpp"

namespace cmd {

namespace {

// TODO
// constexpr uint32_t STEPS_PER_REV = 200;

StepperMotor lowering_platform_left(pins::PLATFORM_PINS[0].dir, pins::PLATFORM_PINS[0].step);
StepperMotor lowering_platform_right(pins::PLATFORM_PINS[1].dir, pins::PLATFORM_PINS[1].step, 2);
StepperMotor drill_platform(pins::DRILL_PLATFORM_PINS.dir, pins::DRILL_PLATFORM_PINS.step);

} // namespace

// Moves the platform down 5 revolutions
void platform_up()
{
    // lowering_platform_left.setDirection(StepperMotor::Direction::NEGATIVE);
    lowering_platform_right.setDirection(StepperMotor::Direction::NEGATIVE);

    // lowering_platform_left.setRunState(true);
    lowering_platform_right.setRunState(true);
}

// Moves the platform up for 5 revolutions
void platform_down()
{
    // lowering_platform_left.setDirection(StepperMotor::Direction::POSITIVE);
    lowering_platform_right.setDirection(StepperMotor::Direction::POSITIVE);

    // lowering_platform_left.setRunState(true);
    lowering_platform_right.setRunState(true);
}

void platform_stop()
{
    // lowering_platform_left.setRunState(false);
    lowering_platform_right.setRunState(false);
}

void init_platform()
{
    // lowering_platform_left.setSpeed(5);
    // lowering_platform_right.setSpeed(5);
    // drill_platform.setSpeed(5);
}

void update_platform()
{
    // lowering_platform_left.update();
    lowering_platform_right.update();
    // drill_platform.update();
}

} // namespace cmd