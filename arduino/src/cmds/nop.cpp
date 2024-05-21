#include "nop.hpp"

namespace cmd {

void nop([[maybe_unused]] const String& param)
{
    DEBUG_LOG("Attempting to call non-existant function! (Param = %s)", param.c_str());
    // TODO: Return error code
}

} // namespace cmd