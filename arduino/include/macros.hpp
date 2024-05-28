#pragma once

#ifdef DEBUG
#include <HardwareSerial.h> // Serial
#include <stdio.h>          // snprintf
#include <stdlib.h>         // exit

// NOLINTBEGIN
#define VA_ARGS(...) , ##__VA_ARGS__
#define DEBUG_LOG(msg, ...)                                                                                            \
    {                                                                                                                  \
        Serial.print("[DEBUG]: ");                                                                                       \
        char buf[64]{};                                                                                                \
        snprintf(buf, 64, msg VA_ARGS(__VA_ARGS__));                                                                   \
        Serial.println(buf);                                                                                           \
        Serial.flush();                                                                                                \
    }
#define DEBUG_LOG_HEX(buf, n, msg, ...)                                                                                \
    {                                                                                                                  \
        char strBuf[2 * n + 1]{};                                                                                      \
        for (int i = 0; i < (n); i++) {                                                                                \
            snprintf(&strBuf[i * 2], 3, "%.2x", (buf)[i]);                                                             \
        }                                                                                                              \
        DEBUG_LOG(msg " [0x%s]", strBuf VA_ARGS(__VA_ARGS__))                                                          \
    }
// TODO: Reset program upon assertion failure
#define DEBUG_ASSERT(condition, msg, ...)                                                                              \
    if (!(condition)) {                                                                                                \
        DEBUG_LOG(msg, __VA_ARGS__);                                                                                   \
        void (*resetFunc)() = 0x0;                                                                                     \
        resetFunc();                                                                                                   \
    }
// NOLINTEND
#else
#define DEBUG_LOG(msg, ...)
#define DEBUG_ASSERT(condition, msg, ...)
#endif

constexpr auto CMD_ERR_MSG = "err";