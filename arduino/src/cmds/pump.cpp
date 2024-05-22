#include <Arduino.h>
#include "WString.h"
#include "pump.hpp"

namespace cmd {

namespace {

namespace pump {
const int pumpPinArr[8] = {22, 24, 26, 28, 30, 32, 34, 36};
unsigned long startMillis[8];
uint8_t pumpPeriod[8];
bool pumpsActive[8];
} // namespace pump

void pump_run(uint8_t p_id, uint32_t t)
{
    pump::startMillis[p_id] = millis();
    digitalWrite(pump::pumpPinArr[p_id], HIGH);
    pump::pumpPeriod[p_id] = t;
    pump::pumpsActive[p_id] = true;
}

void pump_stop(uint8_t p_id)
{
    digitalWrite(pump::pumpPinArr[p_id], LOW);
}

} // namespace

String pump_write(PUMP_NUM which, PumpMode mode, const String& param)
{
    int pump_num = static_cast<int>(which);
    
    switch (mode) {
    case PumpMode::START:
        pump_run(pump_num, param.toInt());
        return {};
    case PumpMode::STOP:
        pump_stop(pump_num);
        return {};
    }

    return {};
}

void pump_init()
{
    // set up pumps
    for (int pin : pump::pumpPinArr) {
        pinMode(pin, OUTPUT);
    }

    // pump state to false
    for (auto& iter : pump::pumpsActive) {
        iter = false;
    }

}

void pump_duration()
{
    for (int i = 0; i < 8; i++) {
        if (pump::pumpsActive[i] && ((millis() - pump::startMillis[i]) >= pump::pumpPeriod[i])) {
            digitalWrite(pump::pumpPinArr[i], LOW);
            pump::pumpsActive[i] = false;
        }
    }
#ifdef DEBUG
    Serial.print("Current ms - start ms: ");
    Serial.println(millis() - pump::startMillis[7]); // only prints debugging message for pump id = 0;
#endif
}
} // namespace cmd