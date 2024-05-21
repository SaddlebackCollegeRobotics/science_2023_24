#pragma once

#include "cmds/command_types.hpp"
#include <WString.h>
#include <macros.hpp>

namespace cmd {

// #pragma pack()
struct CommandData
{
    String dev;
    String func;
    String param;
    checksum_data_t checksum;
};

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
    static ParsedCommand from_str(const char* str);

    [[nodiscard]] CommandData get_data() const
    {
        DEBUG_ASSERT(err_ == ParserError::NONE, "Attempted to get parser data after erroneous parse! (Err = %d)",
                     static_cast<int>(err_));

        return data_;
    }
    [[nodiscard]] ParserError get_error() const { return err_; };
    void call(bool auto_respond = true) const;
    void respond(const String& msg) const;

private:
    CommandData data_;
    ParserError err_;

    ParsedCommand(const CommandData& data, ParserError err);
};

} // namespace cmd