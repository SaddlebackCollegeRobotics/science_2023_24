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

const int scoopLevel = 1600;
const int scoopDown = 680;
const int scoopUp = 1750;

void scoop_down(Servo& scoop)
{
    scoop.writeMicroseconds(scoopDown);
}

void scoop_up(Servo& scoop)
{
    scoop.writeMicroseconds(scoopUp);
}

void scoop_level(Servo& scoop)
{
    scoop.writeMicroseconds(scoopLevel);
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
    case ScoopMode::LEVEL:
        scoop_level(scoops[scoop_num]);
        return {};
    }

    return {};
}

void scoop_init()
{
    for (int i = 0; i < scoops.size(); ++i) {
        scoops[i].attach(pins::SCOOP_PINS[i]);
    }

    for (int i = 0; i < scoops.size(); ++i) {
        scoops[i].writeMicroseconds(scoopLevel);
    }
}

} // namespace cmd