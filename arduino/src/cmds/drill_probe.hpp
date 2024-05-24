#pragma once

#include "WString.h"

namespace cmd {

String drill_probe_up(const String& /*unused*/);
String drill_probe_down(const String& /*unused*/);
String drill_probe_adjust(const String& angle);

void drill_probe_init();

} // namespace cmd