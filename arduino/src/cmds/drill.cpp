#include "cmds/drill.hpp"
#include "Arduino.h"
#include "WString.h"
#include "macros.hpp"
#include "pin_defines.hpp"
#include "task_queue.hpp"

namespace cmd {

namespace {

constexpr auto DELAY_MILLIS = 2000ul;

void drill_spin_cw()
{
    task_timer.in(DELAY_MILLIS, [](void*) -> bool {
        DEBUG_LOG("Drill spinning CW.");
        digitalWrite(pins::DRILL.clockwise, HIGH);
        digitalWrite(pins::DRILL.counter_clockwise, LOW);
        return true;
    });
}

void drill_spin_ccw()
{
    task_timer.in(DELAY_MILLIS, [](void*) -> bool {
        DEBUG_LOG("Drill spinning CCW.");
        digitalWrite(pins::DRILL.clockwise, LOW);
        digitalWrite(pins::DRILL.counter_clockwise, HIGH);
        return true;
    });
}

} // namespace

String drill_spin(const String& direction)
{
    if (direction.equals("cw")) {
        drill_spin_cw();
    } else if (direction.equals("ccw")) {
        drill_spin_ccw();
    } else {
        return CMD_ERR_MSG;
    }

    return {};
}

String drill_stop(const String&)
{
    digitalWrite(pins::DRILL.clockwise, LOW);
    digitalWrite(pins::DRILL.counter_clockwise, LOW);
}

void drill_init()
{
    pinMode(pins::DRILL.clockwise, OUTPUT);
    pinMode(pins::DRILL.counter_clockwise, OUTPUT);
}

} // namespace cmd