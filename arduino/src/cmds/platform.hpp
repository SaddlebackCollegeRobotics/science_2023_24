#pragma once

#include "WString.h"
#include "command_types.hpp"

namespace cmd {

void platform_up();
void platform_down();
void platform_stop();
String read_platform_steps(const String& /*unused*/);
String set_platform_overwrite(const String& mode);

// TODO: Check TOF sensor distance for platform estimations.

void init_platform();
void update_platform();

} // namespace cmd