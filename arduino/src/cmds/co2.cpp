#include "co2.hpp"
#include "WString.h"
#include "pin_defines.hpp"
#include "util/map.hpp"
#include "util/strings.hpp"
#include <Adafruit_SCD30.h> // CO2 sensors
#include <Wire.h>
#include <macros.hpp>

namespace cmd {

namespace {

void mux_write(uint8_t data)
{
    Wire.beginTransmission(pins::I2C_MUX_ADDRESS);
    Wire.write(data);
    Wire.endTransmission();
}

void mux_set_channel(uint8_t channel)
{
    // assume Wire is already set up.
    // otherwise Wire.begin();
    mux_write(1 << channel);
}

void mux_reset()
{
    mux_write(0);
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
Adafruit_SCD30 scd30;

// Negative values represent an error
String scd30_get_data()
{
    if (scd30.dataReady()) {
        DEBUG_LOG("SCD30 dat available!");

        if (!scd30.read()) {
            DEBUG_LOG("Failed to read SCD30 data!");
            return "";
        }

        return String(scd30.CO2) + '|' + String(scd30.temperature) + '|' + String(scd30.relative_humidity);
    }

    DEBUG_LOG("No SCD30 sensor data available (type = %d)!");
    return "";
}

} // namespace

String co2_read(CO2_NUM which, [[maybe_unused]] const String& param)
{
    auto channel = static_cast<int>(which);
    if (channel > 7) {
        DEBUG_LOG("Invalid SCD30 mux channel selected (%d)", channel);
        return CMD_ERR_MSG;
    }

    // Only one object, as all sensors are multiplexed on a single I2C line
    mux_set_channel(channel);

    auto ret = scd30_get_data();

    mux_reset();

    return ret;
}

void co2_init()
{
    // Iterate through the 8 sensor multiplexed channels
    // to begin config files.
    for (int i = 0; i < 8; ++i) {
        mux_set_channel(i);
        if (scd30.begin()) {
            // Enable continuous measurement, where we can just query for data,
            // not needing to request it each time.
            if (!scd30.setMeasurementInterval(2)) {
                DEBUG_LOG("SCD30 continuous measurement fault (channel = %d)", i);
            }
        } else {
            DEBUG_LOG("Failed to initialize SCD30 sensor (channel = %d)", i);
        }
    }
}

} // namespace cmd