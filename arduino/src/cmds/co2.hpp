#pragma once

#include "WString.h"

namespace cmd {

enum class CO2_NUM
{
    // TODO: Get pin nums
    CO2_1,
    CO2_2,
    CO2_3,
    CO2_4,
    CO2_5,
    CO2_6,
    CO2_7,
    CO2_8,
};

String co2_read(CO2_NUM which, const String& param);

void co2_init();

} // namespace cmd