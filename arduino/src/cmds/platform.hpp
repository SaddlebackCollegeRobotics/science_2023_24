#pragma once

#include "command_types.hpp"

namespace cmd {

void platform_up();
void platform_down();
void platform_stop();

// String step(const String&);

void init_platform();
void update_platform();

} // namespace cmd