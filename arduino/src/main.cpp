#include "command_parser.hpp"
#include "task_queue.hpp"
#include <Arduino.h>
#include <macros.hpp>

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    if (Serial.available() >= cmd::min_size) {
        auto str = Serial.readStringUntil('\n');
        auto command = cmd::ParsedCommand::from_str(str.c_str());

        if (command.get_error() == cmd::ParserError::NONE) {
            command.call();
        }
    }

    tasks::process();
}