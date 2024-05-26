#include "cmds/drill.hpp"
#include "Arduino.h"
#include "WString.h"
#include "macros.hpp"
#include "pin_defines.hpp"
#include "task_queue.hpp"

namespace cmd {

namespace {

constexpr auto DELAY_MILLIS = 3000ul;

void drill_spin_cw()
{
    digitalWrite(pins::DRILL.counter_clockwise, LOW);
    task_timer.in(DELAY_MILLIS, [](void*) -> bool {
        DEBUG_LOG("Drill spinning CW.");
        digitalWrite(pins::DRILL.clockwise, HIGH);
        return true;
    });
}

void drill_spin_ccw()
{
    digitalWrite(pins::DRILL.clockwise, LOW);
    task_timer.in(DELAY_MILLIS, [](void*) -> bool {
        DEBUG_LOG("Drill spinning CCW.");
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
    DEBUG_LOG("Stopping drill (pins = %d : %d)", pins::DRILL.clockwise, pins::DRILL.counter_clockwise);
    digitalWrite(pins::DRILL.clockwise, LOW);
    digitalWrite(pins::DRILL.counter_clockwise, LOW);

    return {};
}

void drill_init()
{
    pinMode(pins::DRILL.clockwise, OUTPUT);
    pinMode(pins::DRILL.counter_clockwise, OUTPUT);
    digitalWrite(pins::DRILL.clockwise, LOW);
    digitalWrite(pins::DRILL.counter_clockwise, LOW);
}

} // namespace cmd