#pragma once

#include "WString.h"
#include "command_types.hpp"

namespace cmd {

void drill_platform_up();
void drill_platform_down();
void drill_platform_stop();
String set_drill_platform_limit_overwrite(const String& mode);
String set_drill_platform_hard_stop_overwrite(const String& mode);

void init_drill_platform();
void update_drill_platform();

} // namespace cmd