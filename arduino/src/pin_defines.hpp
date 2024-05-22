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

struct stepper_pin_t
{
    pin_t dir;
    pin_t step;
};

constexpr util::array<stepper_pin_t, 2> PLATFORM_PINS = {
    {
        {.dir = 9, .step = 8},
        {.dir = 11, .step = 10},
    },
};

constexpr struct
{
    pin_t left;
    pin_t right;
} PLATFORM_LIMIT_SWITCHES{40, 42};

constexpr int DRILL_PLATFORM_LIMIT_SWITCH = 44;

constexpr stepper_pin_t DRILL_PLATFORM_PINS = {.dir = 13, .step = 12};

const util::array PUMP_PINS = {22, 24, 26, 28, 30, 32, 34, 36};
// TODO: Steppers, platform,

} // namespace pins