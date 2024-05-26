#pragma once

#include "WString.h"
#include "command_types.hpp"

namespace cmd {

void platform_up();
void platform_down();
void platform_stop();
String platform_set_enabled(const String& is_enabled);
String read_platform_revs(const String& /*unused*/);
String set_platform_limit_overwrite(const String& mode);
String set_platform_hard_stop_overwrite(const String& mode);

// TODO: Check TOF sensor distance for platform estimations.

void init_platform();
void update_platform();

} // namespace cmd