#include "command_parser.hpp"
#include <Arduino.h>
#include <defines.hpp>

#ifdef DEBUG
// #include "avr8-stub.h"
#endif

void setup()
{
    Serial.begin(9600);
#ifdef DEBUG
    // debug_init();
#endif
}

void loop()
{
    if (Serial.available() >= cmd::cmd_size) {
        char buf[cmd::cmd_size];
        Serial.readBytes(buf, cmd::cmd_size);
        auto command = cmd::ParsedCommand::from_bytes(buf);

        command.call();
    }
}