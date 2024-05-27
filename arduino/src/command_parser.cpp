#include "command_parser.hpp"
#include "HardwareSerial.h"
#include "cmds/all.hpp"
#include "cmds/command_types.hpp"
#include "util/array.hpp"
#include "util/misc.hpp"
#include "util/strings.hpp"
#include <CRC32.h>

namespace cmd {

ParsedCommand ParsedCommand::from_str(const char* str)
{
    CommandData data{};
    ParserError err = ParserError::NONE;

    auto str_data = util::split<4>(str);

    data.dev = str_data[0];
    data.func = str_data[1];
    data.param = str_data[2];
    // HACK: str *should* be null-terminated, but we should do checks...
    data.checksum = strtoul(str_data[3].begin(), nullptr, 16);

    // TODO: Refactor. Maybe new macro
    DEBUG_LOG(R"(Parsing command: (dev="%s", func="%s", param="%s", checksum="%#.8lx"))", data.dev.c_str(),
              data.func.c_str(), data.param.c_str(), data.checksum)
// Calc the checksum of the received data, excluding the received checksum itself
#ifdef NO_CHECKSUM
    [[maybe_unused]]
#endif
    checksum_data_t checksum_calc =
        CRC32::calculate(str, str_data[0].length() + str_data[1].length() + str_data[2].length() + 3);

#ifndef NO_CHECKSUM
    // Check whether the sums match
    if (checksum_calc != data.checksum) {
        DEBUG_LOG("Checksum mismatch :: Expected = %#.8lx, Received = %#.8lx", checksum_calc, data.checksum);
        err = ParserError::CHECKSUM_MISMATCH;
        return {data, err};
    }
#endif

    if (COMMAND_MAP[data.dev] == nullptr) {
        DEBUG_LOG("Invalid device name (\"%s\")", data.dev.c_str());
        err = ParserError::INVALID_DEVICE;
        return {data, err};
    }

    // COMMAND_MAP[data.func] is guaranteed to be a valid address,
    // given that the above test passed.
    if ((*COMMAND_MAP[data.dev])[data.func] == nullptr) {
        DEBUG_LOG("Invalid function name (device=\"%s\", function=\"%s\")", data.dev.c_str(), data.func.c_str());
        err = ParserError::INVALID_FUNCTION;
        return {data, err};
    }

    DEBUG_LOG("Successfully parsed command!");

    return {data, err};
}

ParsedCommand::ParsedCommand(const CommandData& data, ParserError err)
    : data_(data)
    , err_(err)
{}

void ParsedCommand::call(bool auto_respond) const
{
    if (err_ != ParserError::NONE) {
        DEBUG_LOG("Attempted to call command with erroneous parse! (Err = %d)", static_cast<int>(err_));
        return;
    }

    DEBUG_LOG(R"(Executing command (dev="%s", func="%s", param="%s"))", data_.dev.c_str(), data_.func.c_str(),
              data_.param.c_str());

    auto func = *(*COMMAND_MAP[data_.dev])[data_.func];

    auto ret = func(data_.param);

    if (auto_respond) {
        respond(ret);
    }
}

void ParsedCommand::respond(const String& msg) const
{
    String response = data_.dev + "," + data_.func + "," + msg + ",";

    auto checksum = CRC32::calculate(response.c_str(), response.length());

    response += String(checksum, HEX);

    response += '\n';

    Serial.print(response);
}

} // namespace cmd