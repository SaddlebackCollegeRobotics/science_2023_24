#pragma once

#include "cmds/co2.hpp"
#include "cmds/command_types.hpp"
#include "cmds/ping.hpp"
#include "cmds/platform.hpp"
#include "cmds/scoop.hpp"
#include "cmds/tof.hpp"
#include "util/map.hpp"
#include "util/strings.hpp"
#include <macros.hpp>
#include <util/array.hpp>
#include <util/pair.hpp>

namespace cmd {

constexpr int NUM_DEVICES = 16;
constexpr int MAX_NUM_FUNCTIONS = 4;
using fn_map = util::map<util::string_view, command_fn_t, MAX_NUM_FUNCTIONS>;
using cmd_map = util::map<util::string_view, fn_map, NUM_DEVICES>;

// TODO: Do we want to be able to select multiple devices?

#define CO2_MAP_ITEM(n)                                                                                                \
    {                                                                                                                  \
        "co2_" #n,                                                                                                     \
            {                                                                                                          \
                {                                                                                                      \
                    {"read_co2",                                                                                       \
                     [](const String& str) -> String { return co2_read(CO2_NUM::CO2_##n, Scd30DataType::CO2, str); }}, \
                    {"read_temp",                                                                                      \
                     [](const String& str) -> String {                                                                 \
                         return co2_read(CO2_NUM::CO2_##n, Scd30DataType::TEMPERATURE, str);                           \
                     }},                                                                                               \
                    {"read_humid",                                                                                     \
                     [](const String& str) -> String {                                                                 \
                         return co2_read(CO2_NUM::CO2_##n, Scd30DataType::HUMIDITY, str);                              \
                     }},                                                                                               \
                },                                                                                                     \
            },                                                                                                         \
    }
#define TOF_MAP_ITEM(n)                                                                                                \
    {                                                                                                                  \
        "tof_" #n,                                                                                                     \
            {                                                                                                          \
                {                                                                                                      \
                    {"read", [](const String& str) -> String { return tof_read(TOF_NUM::TOF_##n, str); }},             \
                },                                                                                                     \
            },                                                                                                         \
    }
// Read / write // TODO: Complete
#define SCOOP_MAP_ITEM(n)                                                                                              \
    {                                                                                                                  \
        "scoop_" #n,                                                                                                   \
            {                                                                                                          \
                {                                                                                                      \
                    {"read", [](const String& str) -> String { return scoop_read(SCOOP_NUM::SCOOP_##n, str); }},       \
                    {"up",                                                                                             \
                     [](const String& str) -> String {                                                                 \
                         return scoop_write(SCOOP_NUM::SCOOP_##n, ScoopMode::UP, str);                                 \
                     }},                                                                                               \
                    {"down",                                                                                           \
                     [](const String& str) -> String {                                                                 \
                         return scoop_write(SCOOP_NUM::SCOOP_##n, ScoopMode::DOWN, str);                               \
                     }},                                                                                               \
                },                                                                                                     \
            },                                                                                                         \
    }
// TODO: Figure out pump functions
#define PUMP_MAP_ITEM(n)                                                                                               \
    {                                                                                                                  \
        "pump_" #n,                                                                                                    \
            {                                                                                                          \
                {                                                                                                      \
                    {"start_time", [](const String& str) -> String { return scoop_read(SCOOP_NUM::SCOOP_##n, str); }}, \
                                                                                                                       \
                },                                                                                                     \
            },                                                                                                         \
    }

constexpr cmd_map COMMAND_MAP = {
    {{
         "root",
         {
             {
                 {"ping", ping},
                 // TODO: Hard reset function
             },
         },
     },
     CO2_MAP_ITEM(1),
     CO2_MAP_ITEM(2),
     CO2_MAP_ITEM(3),
     CO2_MAP_ITEM(4),
     CO2_MAP_ITEM(5),
     CO2_MAP_ITEM(6),
     CO2_MAP_ITEM(7),
     CO2_MAP_ITEM(8),
     //
     TOF_MAP_ITEM(1),
     TOF_MAP_ITEM(2),
     //
     SCOOP_MAP_ITEM(1),
     SCOOP_MAP_ITEM(2),
     SCOOP_MAP_ITEM(3),
     SCOOP_MAP_ITEM(4),
     {
         "platform",
         {
             {
                 {"up",
                  [](const String&) {
                      platform_up();
                      return String{};
                  }},
                 {"down",
                  [](const String&) {
                      platform_down();
                      return String{};
                  }},
                 {"stop",
                  [](const String&) {
                      platform_stop();
                      return String{};
                  }},
             },
         },
     }},
};

#undef CO2_MAP_ITEM
#undef TOF_MAP_ITEM
#undef SCOOP_MAP_ITEM
#undef PUMP_MAP_ITEM

inline void init_hooks()
{
    co2_init();
    tof_init();
    scoop_init();
    init_platform();
    // pump_init();
    // stepper_init();
}

inline void update_hooks()
{
    update_platform();
}

} // namespace cmd