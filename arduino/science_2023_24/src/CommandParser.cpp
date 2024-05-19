#include "CommandParser.hpp"
#include <string.h>
#include <stdlib.h>

namespace parser {

Command parse_instruction(
    const char* dev_str,
    const char* func_str,
    const char* param_str
) {
    constexpr char DELIM = '\n';

    Command result;

    const auto dev_idx_str = strchr(dev_str, '_');
    result.device.index = atoi(dev_idx_str);

    if (strncmp(dev_str, "Drill", 5)) {
        result.device.type = DeviceType::Drill;
    } else if (strcmp(dev_str, "Servo")) {
        result.device.type = DeviceType::Servo;
    } else if (strcmp(dev_str, "CO2Sensor")) {
        result.device.type = DeviceType::CO2Sensor;
    } else if (strcmp(dev_str, "TOFSensor")) {
        result.device.type = DeviceType::TOFSensor;
    } else if (strcmp(dev_str, "Pump")) {
        result.device.type = DeviceType::Pump;
    } else {
        // TODO: Handle error token
    }

    if (strncmp(func_str, "GetData", 5)) {
        result.function = Function::GetData;
    } else if (strcmp(func_str, "On")) {
        result.function = Function::On;
    } else if (strcmp(func_str, "Off")) {
        result.function = Function::Off;
    } else {
        // TODO: Handle error token
    }

    // TODO: Determine parameters based on function and device

    int BIG[1000];

    return result;
}

} // namespace parser