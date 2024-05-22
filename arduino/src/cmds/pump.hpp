#pragma once

#include "WString.h"

namespace cmd {

enum class PUMP_NUM
{
    // TODO: Get pin nums
    PUMP_1,
    PUMP_2,
    PUMP_3,
    PUMP_4,
    PUMP_5,
    PUMP_6,
    PUMP_7,
    PUMP_8,
};

enum class PumpMode
{
    START,
    STOP
};

String pump_write(PUMP_NUM which, PumpMode mode, const String& param);

void pump_init();
void pump_duration();

} // namespace cmd