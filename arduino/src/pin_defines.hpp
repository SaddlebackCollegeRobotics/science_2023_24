#pragma once

#include "util/array.hpp"
#include <stdint.h>

namespace pins {

using pin_t = uint8_t;

// I2C address for multiplexing CO2 sensors
constexpr pin_t I2C_MUX_ADDRESS = 0x70;

// Time of Flight sensor addresses
constexpr util::array<pin_t, 2> TOF_LOX_ADDRESSES = {
    0x53,
    0x54,
};

constexpr util::array<pin_t, 4> SCOOP_PINS = {
    0x02,
    0x03,
    0x04,
    0x05,
};

// TODO: Steppers, platform,

} // namespace pins