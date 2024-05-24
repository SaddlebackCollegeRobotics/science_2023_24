#include "root.hpp"
#include "WString.h"
#include <stdlib.h>

namespace cmd {

String ping(const String& param)
{
    return param;
}

String hard_reset(const String& /*unused*/)
{
    void (*first_addr)(void) = 0;

    first_addr();

    // If that doesn't work for some reason, just exit() and let watchdog reset the program
    exit(1);
}

} // namespace cmd