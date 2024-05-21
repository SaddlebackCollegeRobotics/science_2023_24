#pragma once

#include <stdint.h>

namespace cmd {

using dev_id_t = uint8_t;
using fn_id_t = uint8_t;
using checksum_data_t = uint32_t;

// Parameter type of a command: 4 bytes of data
union param_t
{
    uint32_t data;
    uint8_t i8_data;
    uint8_t i16_data;
    int i32_data;
    float f32_data;
};

// Return type of a command: 4 bytes of data
union ret_t
{
    uint32_t data;
    int int_data;
    float float_data;
};

static_assert(sizeof(dev_id_t) == 1, "Device id type must be 1 byte large!");
static_assert(sizeof(fn_id_t) == 1, "Function id type must be 1 byte large!");
static_assert(sizeof(param_t) == 4, "Command parameter type must be 4 bytes large!");
static_assert(sizeof(param_t) == 4, "Command parameter type must be 4 bytes large!");
static_assert(sizeof(ret_t) == 4, "Command return type must be 4 bytes large!");
static_assert(sizeof(checksum_data_t) == 4, "Command checksum data type must be 4 bytes large!");

// Alias for a pointer to an arbitrary command function
using command_fn_t = __attribute__((warn_unused_result)) ret_t (*)(param_t);

} // namespace cmd