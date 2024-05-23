#define DEBUG

#include "StepperMotor/StepperMotor.hpp"
#include "cmds/all.hpp"
#include "command_parser.hpp"
#include "pin_defines.hpp"
#include "task_queue.hpp"
#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>
#include <macros.hpp>

void setup()
{
    Serial.begin(9600);

    cmd::init_hooks();

    Timer1.initialize(2000);

    // Timer1.attachInterrupt([] { Serial.println("INTERRUPTED!"); });
}

void loop()
{
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

    cmd::update_hooks();

    tasks::process();
}