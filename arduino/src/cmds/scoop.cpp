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

const int scoopDown = 550;
const int scoopUp = 1550;

void scoop_down(Servo& scoop)
{
    scoop.writeMicroseconds(scoopDown);
}

void scoop_up(Servo& scoop)
{
    scoop.writeMicroseconds(scoopUp);
}

} // namespace

String scoop_read(SCOOP_NUM which, const String& param)
{
    int scoop_num = static_cast<int>(which);
    return {};
}

String scoop_write(SCOOP_NUM which, ScoopMode mode, const String& param)
{
    int scoop_num = static_cast<int>(which);

    switch (mode) {
    case ScoopMode::UP:
        scoop_up(scoops[scoop_num]);
        return {};
    case ScoopMode::DOWN:
        scoop_down(scoops[scoop_num]);
        return {};
    }

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