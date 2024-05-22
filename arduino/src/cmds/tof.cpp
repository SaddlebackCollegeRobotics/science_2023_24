#include "cmds/tof.hpp"
#include "WString.h"
#include <Adafruit_VL53L0X.h> // ToF sensors
#include <macros.hpp>

namespace cmd {

namespace {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
Adafruit_VL53L0X tof_1;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
Adafruit_VL53L0X tof_2;

void init_tof(Adafruit_VL53L0X& init, uint8_t addr)
{
    if (init.begin(addr)) {
        // Enable continuous range monitoring so that it's not necessary
        // to query every time.
        init.startRangeContinuous();
    } else {
        DEBUG_LOG("Failed to initialize TOF sensor (addr = %.2x)", addr);
    }
}

} // namespace

String tof_read(TOF_NUM which, [[maybe_unused]] const String& param)
{
    Adafruit_VL53L0X* tof{};

    switch (which) {
    case TOF_NUM::TOF_1:
        tof = &tof_1;
        break;
    case TOF_NUM::TOF_2:
        tof = &tof_2;
        break;
    default:
        DEBUG_LOG("Encountered unknown TOF sensor (%d)", static_cast<int>(which));
        return CMD_ERR_MSG;
    }

    // TODO: More complete error handling
    if (tof->isRangeComplete() /*&& tof->readRangeStatus()*/) {
        return String(tof->readRangeResult());
    }

    // Still in the process of reading (or other error...)
    return CMD_ERR_MSG;
}

void tof_init() {}

} // namespace cmd