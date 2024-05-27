#include "pump.hpp"
#include "WString.h"
#include "pin_defines.hpp"
#include <Arduino.h>
#include <macros.hpp>

namespace cmd {

namespace {

void pump_run(uint8_t p_id /*, uint32_t t*/)
{
    DEBUG_LOG("Running pump (pin = %d)", pins::PUMP_PINS[p_id]);
    digitalWrite(pins::PUMP_PINS[p_id], HIGH);
}

void pump_stop(uint8_t p_id)
{
    DEBUG_LOG("Stopping pump (pin = %d)", pins::PUMP_PINS[p_id]);
    digitalWrite(pins::PUMP_PINS[p_id], LOW);
}

} // namespace

String pump_write(PUMP_NUM which, PumpMode mode, [[maybe_unused]] const String& param)
{
    int pump_num = static_cast<int>(which);

    switch (mode) {
    case PumpMode::START:
        pump_run(pump_num);
        return {};
    case PumpMode::STOP:
        pump_stop(pump_num);
        return {};
    default:
        DEBUG_LOG("Invalid pump selected (%d)", pump_num);
        return CMD_ERR_MSG;
    }

    return {};
}

void pump_init()
{
    // set up pumps
    for (int pin : pins::PUMP_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

// void pump_duration()
// {
//     for (int i = 0; i < 8; i++) {
//         if (pump::pumpsActive[i] && ((millis() - pump::startMillis[i]) >= pump::pumpPeriod[i])) {
//             digitalWrite(pins::PUMP_PINS[i], LOW);
//             pump::pumpsActive[i] = false;
//         }
//     }

//     DEBUG_LOG("Current ms - start ms: ()", millis() - pump::startMillis[7]);
// }
} // namespace cmd