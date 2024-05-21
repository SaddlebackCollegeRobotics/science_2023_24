#pragma once

#include "command_types.hpp"
#include "nop.hpp"
#include <defines.hpp>
#include <util/array.hpp>
#include <util/pair.hpp>

namespace cmd {

constexpr int NUM_DEVICES = 2;
constexpr int MAX_NUM_FUNCTIONS = 2;
using fn_list = util::array<command_fn_t, MAX_NUM_FUNCTIONS>;
using cmd_list = util::array<fn_list, NUM_DEVICES>;

// TODO: Do we want to be able to select multiple devices?

constexpr cmd_list COMMAND_MAP = {{
    fn_list::fill(nop),
    {nop, nop},
}};

void init_map();

// #ifdef DEBUG
constexpr util::array<util::pair<const char*, util::array<const char*, 256>>, 256> COMMAND_NAME_MAP = {
    {"None", {"nop"}},
};
// #endif

} // namespace cmd