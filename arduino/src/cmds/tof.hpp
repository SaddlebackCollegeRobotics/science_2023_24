#pragma once

#include "WString.h"

namespace cmd {

enum class TOF_NUM
{
    // TODO: Get pin nums
    TOF_1,
    TOF_2
};

String tof_read(TOF_NUM which, const String& param);

void tof_init();

} // namespace cmd