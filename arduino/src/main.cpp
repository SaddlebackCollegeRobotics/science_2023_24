#include "cmds/all.hpp"
#include "command_parser.hpp"
#include "pin_defines.hpp"
#include "task_queue.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <macros.hpp>

void setup()
{
    Serial.begin(9600);

    cmd::init_all();
    // digitalWrite(pins::PLATFORM_PINS[1].dir, HIGH);
}

void loop()
{
    // digitalWrite(pins::PLATFORM_PINS[1].step, HIGH);
    // delay(100);
    // digitalWrite(pins::PLATFORM_PINS[1].step, LOW);
    // return;

    if (Serial.available() >= cmd::min_size) {
        // auto before = millis();
        auto str = Serial.readStringUntil('\n');
        // DEBUG_LOG("readStringUntil(): took %ld millis", millis() - before);
        // before = millis();

        auto command = cmd::ParsedCommand::from_str(str.c_str());
        // DEBUG_LOG("ParsedCommand::from_str(): took %ld millis", millis() - before);
        // before = millis();

        if (command.get_error() == cmd::ParserError::NONE) {
            command.call();
            // DEBUG_LOG("command.call(): took %ld millis", millis() - before);
        }
    }

    tasks::process();
}