#include "drill_probe.hpp"
#include "Servo.h"
#include "pin_defines.hpp"
#include <util/math.hpp>

namespace cmd {

namespace {

constexpr int DRILL_US_UP = 600;
constexpr int DRILL_US_DOWN = 1000;

Servo drill_probe;

// Accepts a value microseconds to move the servo
void move_drill_to(int us_value)
{
    auto clamped_val = util::clamp(us_value, DRILL_US_DOWN, DRILL_US_UP);
    drill_probe.writeMicroseconds(clamped_val);
}

} // namespace

String drill_probe_up(const String& /*unused*/)
{
    move_drill_to(DRILL_US_UP);
    return {};
}
String drill_probe_down(const String& /*unused*/)
{
    move_drill_to(DRILL_US_DOWN);
    return {};
}
String drill_probe_adjust(const String& angle)
{
    const int delta_degees = static_cast<int>(angle.toInt());
    const auto current_angle_us = drill_probe.readMicroseconds();

    move_drill_to(delta_degees + current_angle_us);

    return {};
}

void drill_probe_init()
{
    drill_probe.attach(pins::DRILL_PROBE);

    drill_probe.writeMicroseconds(DRILL_US_UP);
}

} // namespace cmd