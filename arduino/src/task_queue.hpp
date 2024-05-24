#pragma once

#include <arduino-timer.h>

namespace tasks {

constexpr int MAX_TASKS = 32;

} // namespace tasks

extern Timer<tasks::MAX_TASKS> task_timer;