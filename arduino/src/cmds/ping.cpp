#include "ping.hpp"
#include "WString.h"

namespace cmd {

String ping(const String& param)
{
    return param;
}

String hard_reset(const String&)
{
    void (*first_addr)(void) = 0;

    first_addr();
}

} // namespace cmd