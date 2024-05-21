#pragma once
#define DEBUG
#ifdef DEBUG
#include <HardwareSerial.h> // Serial
#include <stdio.h>          // snprintf
#include <stdlib.h>         // exit

// NOLINTBEGIN
#define VA_ARGS(...) , ##__VA_ARGS__
#define DEBUG_LOG(msg, ...)                                                                                            \
    {                                                                                                                  \
        Serial.print("DEBUG LOG: ");                                                                                   \
        char buf[64]{};                                                                                                \
        snprintf(buf, 64, msg VA_ARGS(__VA_ARGS__));                                                                   \
        Serial.println(buf);                                                                                           \
    }
// TODO: Reset program upon assertion failure
#define DEBUG_ASSERT(condition, msg, ...)                                                                              \
    if (!(condition)) {                                                                                                \
        DEBUG_LOG(msg, __VA_ARGS__);                                                                                   \
        exit(1);                                                                                                       \
    }
// NOLINTEND
#else
#define DEBUG_LOG(msg)
#define DEBUG_ASSERT(condition, msg)
#endif