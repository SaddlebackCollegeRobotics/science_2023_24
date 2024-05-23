#include "task_queue.hpp"
#include "Arduino.h"
#include <arduino-timer.h>

constexpr int MAX_TASKS = 32;

Timer<10> task_timer;

namespace {

using callback_t = void (*)();
using timer_t = Timer<MAX_TASKS, millis, callback_t>;

auto timer = timer_t{};

// TODO: Maybe switch to map for more descriptive access?

bool task_handler_helper(callback_t fn_ptr)
{
    fn_ptr();
    return true;
}

} // namespace

namespace tasks {

void* add_task(unsigned long millis_delay, callback_t callback)
{
    timer_t::Task task_handle = timer.in(millis_delay, task_handler_helper, callback);

    return task_handle;
}

void cancel_task(void* task_handle)
{
    timer.cancel(task_handle);
}

void process()
{
    timer.tick();
    // TODO: Try to handle millis overflow?
}

} // namespace tasks