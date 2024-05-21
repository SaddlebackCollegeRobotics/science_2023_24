#pragma once

namespace tasks {

// Returns: a handle to the added task, for possible cancelation later
void* add_task(unsigned long millis_delay, void (*callback)());

void cancel_task(void* task_handle);

void process();

} // namespace tasks