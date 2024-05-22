#include "platform.hpp"
#include "Arduino.h"
#include "Stepper.h"
#include "WString.h"
#include "pin_defines.hpp"

namespace cmd {

namespace {

// TODO
constexpr uint32_t STEPS_PER_REV = 200;

Stepper lowering_platform_left(STEPS_PER_REV, pins::PLATFORM_PINS[0].step, pins::PLATFORM_PINS[0].dir);
Stepper lowering_platform_right(STEPS_PER_REV, pins::PLATFORM_PINS[1].step, pins::PLATFORM_PINS[1].dir);
Stepper drill_platform(STEPS_PER_REV, pins::DRILL_PLATFORM_PINS.dir, pins::DRILL_PLATFORM_PINS.step);

} // namespace

// Moves the platform down 5 revolutions
void platform_down_test()
{
    constexpr auto rev5 = 5 * STEPS_PER_REV;

    // lowering_platform_left.step(rev5);
    // lowering_platform_right.step(rev5);
}

// Moves the platform up for 5 revolutions
void platform_up_test()
{
    constexpr auto rev5 = 5 * STEPS_PER_REV;

    lowering_platform_left.step(rev5);
    lowering_platform_right.step(rev5);
}

// String stepper(const String& param)
// {
//     return param;
// }

void init_platform()
{
    lowering_platform_left.setSpeed(5);
    lowering_platform_right.setSpeed(5);
    drill_platform.setSpeed(5);
}

} // namespace cmd