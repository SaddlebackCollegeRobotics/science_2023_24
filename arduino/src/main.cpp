#include "HardwareSerial.h"
#include <Arduino.h>
#include <defines.hpp>

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    if (Serial.available() > 10) {
        DEBUG_ASSERT(false, "Too much DATA (%d)", Serial.available());
        DEBUG_LOG("Unreachable (I think)");
    }
}