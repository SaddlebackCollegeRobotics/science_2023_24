#include "scoop.hpp"
#include "WString.h"
#include "pin_defines.hpp"
#include "util/array.hpp"
#include <Servo.h> // PWM module . Stepper module
#include <macros.hpp>

namespace cmd {

namespace {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
util::array<Servo, 4> scoops = {};

} // namespace

String scoop_read(SCOOP_NUM which, const String& param)
{
    return {};
}

String scoop_write(SCOOP_NUM which, const String& param)
{
    return {};
}

void scoop_init()
{
    for (int i = 0; i < scoops.size(); ++i) {
        scoops[i].attach(pins::SCOOP_PINS[i]);
    }

    // TODO: Move scoops up upon initialization?
}

} // namespace cmd