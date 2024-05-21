#include "command_parser.hpp"
#include "cmds/all.hpp"
#include "cmds/command_types.hpp"
#include "util/array.hpp"
#include <CRC32.h>

namespace {
union conversion
{
    cmd::CommandData cmd_data;
    char raw : cmd::cmd_size * 8;
};

} // namespace

namespace cmd {

ParsedCommand ParsedCommand::from_bytes(const char* bytes)
{
    CommandData data = *(CommandData*)bytes;
    Serial.write(bytes, 10);
    Serial.println();
    Serial.write((byte*)(&data), 10);
    Serial.println();

    // memcpy(&data, bytes, sizeof(data));
    ParserError err = ParserError::NONE;
    DEBUG_LOG_HEX(bytes, 10, "Parsing data:");

    DEBUG_LOG_HEX((const uint8_t*)(&data), sizeof(data) - sizeof(checksum_data_t), "Calculating checksum for:");
    // Calc the checksum of the received data, excluding the received checksum itself
    checksum_data_t checksum_calc = CRC32::calculate(&data, sizeof(data) - sizeof(checksum_data_t));

    // Check whether the sums match
    if (checksum_calc != data.checksum) {
        DEBUG_LOG_CMD(data, "Checksum mismatch :: Expected = %#.4lx, Received = %#.4lx", checksum_calc, data.checksum);
        err = ParserError::CHECKSUM_MISMATCH;
        return {data, err};
    }

    if (!in_range(data.dev, COMMAND_MAP)) {
        err = ParserError::INVALID_DEVICE;
        return {data, err};
    }

    // COMMAND_MAP[data.func] is guaranteed to be a valid address,
    // given that the above test passed.
    if (!in_range(data.func, COMMAND_MAP[data.func])) {
        err = ParserError::INVALID_FUNCTION;
        return {data, err};
    }

    return {data, err};
}

ParsedCommand::ParsedCommand(const CommandData& data, ParserError err)
    : data_(data)
    , err_(err)
{}

ret_t ParsedCommand::call() const
{
    if (err_ != ParserError::NONE) {
        DEBUG_LOG_CMD(data_, "Attempted to call command with erroneous parse! (Err = %d)", static_cast<int>(err_));
        return {};
    }

    DEBUG_LOG_CMD(data_, "Executing command");

    auto func = COMMAND_MAP[data_.dev][data_.func];

    return func(data_.param);
}

} // namespace cmd