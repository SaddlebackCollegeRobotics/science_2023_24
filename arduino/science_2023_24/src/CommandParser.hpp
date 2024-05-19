#pragma once


namespace parser {
/* Commands are of the following form:
*   DEVICE_NAME ',' FUNCTION_NAME [',' PARAMETER]+ '\n'
*
*  Tokens may contain any utf-8 character other than a ','
*  Specific valid token names may be found in the enums below
*  Names are case and space sensitive
*/

enum class DeviceType
{
    Drill,
    Servo,
    CO2Sensor,
    TOFSensor, // Time of Flight
    Pump
};

/* A valid DEVICE_NAME token is of the following form:
*   DEVICE_TYPE '_' INDEX
*
*   where DEVICE_TYPE is one of the names above,
*   and INDEX is the 0-indexed decimal number of the device.
*
*   Examples: "Drill_0" "Pump_4"
*/
struct Device
{
    DeviceType type;
    int index;
};

/* A valid FUNCTION_NAME token is simply one of the names below.
*
*   Examples: "GetData" "On"
*/
enum class Function
{
    GetData,
    On,
    Off,
    // TODO
};

/* Valid PARAMETER tokens are documented with their associated function calls.
!!!TODO
*/
constexpr int MAX_PARAMS = 3;
union Parameter
{
    bool value_b;
    int value_i;
    float value_f;
    double value_d;
};

struct Command
{
    Device device;
    Function function;
    Parameter params[MAX_PARAMS];
};

Command parse_instruction(
    const char* dev_str,
    const char* func_str,
    const char* param_str
);

} // namespace parser