#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "pins.hpp"
#include "CommandFn.hpp"

#include <map.hpp>
#include <array.hpp>
#include <functional.hpp>
#include <tuple.hpp>
#include <misc.hpp>

void setup() {
  Serial.begin(9600);
  map(10, 10, 10, 10, 10);
}


void add(int x, int y)
{
  Serial.print("Adding things = ");
  Serial.println(x + y);
}

constexpr util::map<const char*, util::map<const char*, CommandFn, 1>, 1> command_map {
    {"dev", {"add", add}},
};
auto add_fn = CommandFn{add};

void loop() {

  // command_map["on"] = 5;
  char buffer[33] = {};
  if (Serial.available()) {
    auto str = Serial.readString();
    str.trim();
    auto split_str = util::split<2>(str.c_str());
    Serial.print(("Read : '" + str + "'\n").c_str());
    Serial.print("Command is: ");
    Serial.print(split_str[0]);
    Serial.print(" : ");
    Serial.println(split_str[1]);
  }
  delay(1000);
  Serial.write("Waiting...\n");
}