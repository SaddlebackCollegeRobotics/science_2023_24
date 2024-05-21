#pragma once

#ifdef DEBUG
#include <stdlib.h> // exit

#define DEBUG_LOG(msg)                                                                                                 \
    {                                                                                                                  \
        Serial.print("DEBUG LOG: ");                                                                                   \
        Serial.println(msg);                                                                                           \
    }
#define DEBUG_ASSERT(condition, msg)                                                                                   \
    if (!(condition)) {                                                                                                \
        DEBUG_LOG(msg);                                                                                                \
        exit(1);                                                                                                       \
    }
#else
#define DEBUG_LOG(msg)
#define DEBUG_ASSERT(condition, msg)
#endif