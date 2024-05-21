#pragma once

#include "cmds/all.hpp"
#include "cmds/command_types.hpp"
#include <stdint.h>

namespace cmd {

struct CommandData
{
    uint8_t dev;
    uint8_t func;
    param_t param;
    checksum_data_t checksum;
};

static_assert(sizeof(CommandData) == 10, "CommandData must be 10 bytes large!");

enum class ParserError
{
    NONE,
    INVALID_DEVICE,
    INVALID_FUNCTION,
    CHECKSUM_MISMATCH
};

class ParsedCommand
{
public:
    static ParsedCommand from_bytes(const char* bytes);

    [[nodiscard]] CommandData get_data() const
    {
        DEBUG_ASSERT(err_ == ParserError::NONE, "Attempted to get parser data after erroneous parse! (Err = %d)",
                     static_cast<int>(err_));

        return data_;
    }
    [[nodiscard]] ParserError get_error() const { return err_; };
    [[nodiscard]] ret_t call() const;

private:
    CommandData data_;
    ParserError err_;

    ParsedCommand(const CommandData& data, ParserError err);
};

} // namespace cmd