#pragma once

#include "command_types.hpp"
#include "nop.hpp"
#include "ping.hpp"
#include "util/map.hpp"
#include "util/strings.hpp"
#include <macros.hpp>
#include <util/array.hpp>
#include <util/pair.hpp>

namespace cmd {

constexpr int NUM_DEVICES = 2;
constexpr int MAX_NUM_FUNCTIONS = 2;
using fn_map = util::map<util::string_view, command_fn_t, MAX_NUM_FUNCTIONS>;
using cmd_map = util::map<util::string_view, fn_map, NUM_DEVICES>;

// TODO: Do we want to be able to select multiple devices?

constexpr cmd_map COMMAND_MAP = {
    {
        "mega",
        {{"ping", ping}},
    },
};

void init_map();

// #ifdef DEBUG
constexpr util::array<util::pair<const char*, util::array<const char*, 256>>, 256> COMMAND_NAME_MAP = {
    {"None", {"nop"}},
};
// #endif

} // namespace cmd