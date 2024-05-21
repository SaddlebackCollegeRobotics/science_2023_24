#include "nop.hpp"

namespace cmd {

ret_t nop([[maybe_unused]] param_t param)
{
    DEBUG_LOG("Attempting to call non-existant function! (Param = %d)", param);
    // TODO: Return error code
    return {};
}

} // namespace cmd