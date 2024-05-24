#include "drill_sensors.hpp"
#include "Arduino.h"
#include "WString.h"
#include "pin_defines.hpp"
#include <math.h>

namespace cmd {

namespace {

// Code for temperature sensor AD88495:
// https://learn.adafruit.com/ad8495-thermocouple-amplifier/arduino

constexpr float AREF = 5;          // set to AREF, typically board voltage like 3.3 or 5.0
constexpr int ADC_RESOLUTION = 10; // set to ADC bit resolution, 10 is default

float get_voltage(int raw_adc)
{
    return raw_adc * (AREF / ((1 << ADC_RESOLUTION) - 1));
}

float get_temperature(float voltage)
{
    return (voltage - 1.25) / 0.005;
}
} // namespace

String get_drill_temp(const String&)
{
    const int analog_reading = analogRead(pins::TEMP_SENSOR_PIN);
    const float voltage = get_voltage(analog_reading);
    // HACK: Tested offset
    const float temp = get_temperature(voltage) + 2.3;

    return String(temp);
}

String get_drill_moisture(const String&)
{
    const int analog_reading = analogRead(pins::MOISTURE_SENSOR_PIN);

    return String(analog_reading);
}

void drill_sensors_init()
{
    pinMode(pins::TEMP_SENSOR_PIN, INPUT);
    pinMode(pins::MOISTURE_SENSOR_PIN, INPUT);
}

} // namespace cmd