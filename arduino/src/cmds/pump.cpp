#include "pump.hpp"
#include "WString.h"
#include "pin_defines.hpp"
#include <Arduino.h>
#include <macros.hpp>

namespace cmd {

namespace {

namespace pump {
// unsigned long startMillis[8];
// uint8_t pumpPeriod[8];
// bool pumpsActive[8];
} // namespace pump

void pump_run(uint8_t p_id /*, uint32_t t*/)
{
    // pump::startMillis[p_id] = millis();
    DEBUG_LOG("Running pump (pin = %d)", pins::PUMP_PINS[p_id]);
    digitalWrite(pins::PUMP_PINS[p_id], HIGH);
    // pump::pumpPeriod[p_id] = t;
    // pump::pumpsActive[p_id] = true;
}

void pump_stop(uint8_t p_id)
{
    DEBUG_LOG("Stopping pump (pin = %d)", pins::PUMP_PINS[p_id]);
    digitalWrite(pins::PUMP_PINS[p_id], LOW);
}

} // namespace

String pump_write(PUMP_NUM which, PumpMode mode, const String& param)
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

    // pump state to false
    // for (auto& iter : pump::pumpsActive) {
    //     iter = false;
    // }
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