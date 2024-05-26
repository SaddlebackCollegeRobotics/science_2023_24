#pragma once

#include "cmds/co2.hpp"
#include "cmds/command_types.hpp"
#include "cmds/drill.hpp"
#include "cmds/drill_platform.hpp"
#include "cmds/drill_probe.hpp"
#include "cmds/drill_sensors.hpp"
#include "cmds/platform.hpp"
#include "cmds/pump.hpp"
#include "cmds/root.hpp"
#include "cmds/scoop.hpp"
#include "cmds/tof.hpp"
#include "util/map.hpp"
#include "util/strings.hpp"
#include <macros.hpp>
#include <util/array.hpp>
#include <util/pair.hpp>

namespace cmd {

constexpr int NUM_DEVICES = 26;
constexpr int MAX_NUM_FUNCTIONS = 7;
using fn_map = util::map<util::string_view, command_fn_t, MAX_NUM_FUNCTIONS>;
using cmd_map = util::map<util::string_view, fn_map, NUM_DEVICES>;

#define CO2_MAP_ITEM(n)                                                                                                \
    {                                                                                                                  \
        "co2_" #n,                                                                                                     \
            {                                                                                                          \
                {                                                                                                      \
                    {"read",                                                                                       \
                     [](const String& str) -> String { return co2_read(CO2_NUM::CO2_##n, str); }},                                                                                             \
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
#define SCOOP_MAP_ITEM(n)                                                                                              \
    {                                                                                                                  \
        "scoop_" #n,                                                                                                   \
            {                                                                                                          \
                {                                                                                                      \
                    {"up",                                                                                             \
                     [](const String& str) -> String {                                                                 \
                         return scoop_write(SCOOP_NUM::SCOOP_##n, ScoopMode::UP, str);                                 \
                     }},                                                                                               \
                    {"down",                                                                                           \
                     [](const String& str) -> String {                                                                 \
                         return scoop_write(SCOOP_NUM::SCOOP_##n, ScoopMode::DOWN, str);                               \
                     }},                                                                                               \
                    {"level",                                                                                          \
                     [](const String& str) -> String {                                                                 \
                         return scoop_write(SCOOP_NUM::SCOOP_##n, ScoopMode::LEVEL, str);                              \
                     }},                                                                                               \
                    {"adjust",                                                                                         \
                     [](const String& str) -> String {                                                                 \
                         return scoop_write(SCOOP_NUM::SCOOP_##n, ScoopMode::ADJUST, str);                             \
                     }},                                                                                               \
                },                                                                                                     \
            },                                                                                                         \
    }
#define PUMP_MAP_ITEM(n)                                                                                               \
    {                                                                                                                  \
        "pump_" #n,                                                                                                    \
            {                                                                                                          \
                {                                                                                                      \
                    {"start",                                                                                          \
                     [](const String& str) -> String {                                                                 \
                         return pump_write(PUMP_NUM::PUMP_##n, PumpMode::START, str);                                  \
                     }},                                                                                               \
                    {"stop",                                                                                           \
                     [](const String& str) -> String { return pump_write(PUMP_NUM::PUMP_##n, PumpMode::STOP, str); }}, \
                                                                                                                       \
                },                                                                                                     \
            },                                                                                                         \
    }

constexpr cmd_map COMMAND_MAP = {
    {
        {
            "root",
            {
                {
                    {"ping", ping},
                    {"reset", hard_reset},
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
        // Platform management
        {
            "platform_lower",
            {{{"up",
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
              {
                  "limit_input",
                  set_platform_limit_overwrite,
              },
              {
                  "hard_stop_input",
                  set_platform_hard_stop_overwrite,
              },
              {
                  "read",
                  read_platform_revs,
              },
              {
                  "set_enabled",
                  platform_set_enabled,
              },
              }},
              
        },
        //
        {
            "drill_lower",
            {{{"up",
               [](const String&) {
                   drill_platform_up();
                   return String{};
               }},
              {"down",
               [](const String&) {
                   drill_platform_down();
                   return String{};
               }},
              {"stop",
               [](const String&) {
                   drill_platform_stop();
                   return String{};
               }},
              {
                  "limit_input",
                  set_drill_platform_overwrite,
              }}},
        },
        //
        PUMP_MAP_ITEM(1),
        PUMP_MAP_ITEM(2),
        PUMP_MAP_ITEM(3),
        PUMP_MAP_ITEM(4),
        PUMP_MAP_ITEM(5),
        PUMP_MAP_ITEM(6),
        // Drill
        {
            "drill",
            {{
                {"spin", drill_spin},
                {"stop", drill_stop},
            }},
        },
        // Drill sensors
        {
            "drill_sensors",
            {{
                {"read_temp", get_drill_temp},
                {"read_moisture", get_drill_moisture},
            }},
        },
        // Drill probe
        {"drill_probe",
         {{
             {"up", drill_probe_up},
             {"down", drill_probe_down},
             {"adjust", drill_probe_adjust},
         }}},
    },
};

#undef CO2_MAP_ITEM
#undef TOF_MAP_ITEM
#undef SCOOP_MAP_ITEM
#undef PUMP_MAP_ITEM

// TODO: Handle these hooks better with constexpr, or macros
// TODO: Better map registration. Maybe a class; overhead is negligible.

inline void init_hooks()
{
    co2_init();
    tof_init();
    scoop_init();
    init_platform();
    init_drill_platform();
    drill_probe_init();
    drill_init();
    // pump_init();
    // stepper_init();
}

inline void update_hooks()
{
    update_platform();
    update_drill_platform();
}

} // namespace cmd