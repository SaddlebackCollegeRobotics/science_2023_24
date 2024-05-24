#include "task_queue.hpp"
#include <arduino-timer.h>

Timer<tasks::MAX_TASKS> task_timer;