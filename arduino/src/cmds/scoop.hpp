#pragma once

#include "WString.h"

namespace cmd {

enum class SCOOP_NUM
{
    // TODO: Get pin nums
    SCOOP_1,
    SCOOP_2,
    SCOOP_3,
    SCOOP_4,
};

String scoop_read(SCOOP_NUM which, const String& param);
String scoop_write(SCOOP_NUM which, const String& param);

void scoop_init();

} // namespace cmd