#pragma once

// Stepper motors
namespace drill {

struct drill_pin_t
{
    int DIR;
    int STEP;
};

constexpr struct {
    drill_pin_t platform = {13, 12}; // Name?
    drill_pin_t lowerRight = {9, 8};
    drill_pin_t lowerLeft = {11, 10};
} PINS;

} // namespace drill

// Servos
namespace scoop {
    constexpr int PINS[] = {
        2,
        3,
        4,
        5
    };
} // namespace scoop

//???
namespace pumps {
    constexpr int PINS[] = {
        22,
        24,
        26,
        28,
        30,
        32,
        34,
        36
    };
} // namespace pumps

// I2C Pins
namespace sensors {
    constexpr struct {
        int SDA = 20;
        int SDL = 21;
    } PINS;
} // namespace sensors