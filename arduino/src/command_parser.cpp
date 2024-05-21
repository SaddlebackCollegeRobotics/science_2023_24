#include "command_parser.hpp"
#include "cmds/all.hpp"
#include "cmds/command_types.hpp"
#include "util/array.hpp"
#include <CRC32.h>

namespace cmd {

ParsedCommand ParsedCommand::from_bytes(const char* bytes)
{
    CommandData data = *reinterpret_cast<const CommandData*>(bytes);
    ParserError err = ParserError::NONE;

    // Calc the checksum of the received data, excluding the received checksum itself
    checksum_data_t checksum_calc = CRC32::calculate(&data, sizeof(data) - sizeof(checksum_data_t));

    // Check whether the sums match
    if (checksum_calc != data.checksum) {
        err = ParserError::CHECKSUM_MISMATCH;
        goto ret;
    }

    if (!in_range(data.dev, COMMAND_MAP)) {
        err = ParserError::INVALID_DEVICE;
        goto ret;
    }

    // COMMAND_MAP[data.func] is guaranteed to be a valid address,
    // given that the above test passed.
    if (!in_range(data.func, COMMAND_MAP[data.func])) {
        err = ParserError::INVALID_FUNCTION;
        goto ret;
    }

ret:
    return {data, err};
}

ParsedCommand::ParsedCommand(const CommandData& data, ParserError err)
    : data_(data)
    , err_(err)
{}

ret_t ParsedCommand::call() const
{
    if (err_ != ParserError::NONE) {
        DEBUG_LOG("Attempted to call command with erroneous parse! (Err = %d, Dev = %d, Func = %d)", static_cast<int>(err_), data_.dev,
                  data_.func);
        return {};
    }

    DEBUG_LOG("Executing command (Device = %d, Func = %d, Param = %d)", data_.dev, data_.func, data_.param);

    auto func = COMMAND_MAP[data_.dev][data_.func];

    return func(data_.param);
}

} // namespace cmd