#include "scoop.hpp"
#include "WString.h"
#include "pin_defines.hpp"
#include "util/array.hpp"
#include <Servo.h> // PWM module . Stepper module
#include <macros.hpp>

namespace cmd {

namespace {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
util::array<Servo, 4> scoops = {};

// SCOOP 1 pulse widths
const int scoop1Level = 1600;
const int scoop1Down = 680;
const int scoop1Up = 1750;

// SCOOP 2 pulse widths
const int scoop2Level = 1430;
const int scoop2Down = 2470;
const int scoop2Up = 1300;

// TODO: test real scoops 3 and 4 to determine correct pulse widths
const int scoop3Level = 0;
const int scoop3Down = 0;
const int scoop3Up = 0;
const int scoop4Level = 0;
const int scoop4Down = 0;
const int scoop4Up = 0;

void scoop_down(Servo& scoop, SCOOP_NUM scoopNum)
{
    switch(scoopNum)
    {
        case SCOOP_NUM::SCOOP_1:
        scoop.writeMicroseconds(scoop1Down);
        break;
        case SCOOP_NUM::SCOOP_2:
        scoop.writeMicroseconds(scoop2Down);
        break;
        case SCOOP_NUM::SCOOP_3:
        scoop.writeMicroseconds(scoop3Down);
        break;
        case SCOOP_NUM::SCOOP_4:
        scoop.writeMicroseconds(scoop4Down);
        break;
    }
}

void scoop_up(Servo& scoop, SCOOP_NUM scoopNum)
{
    switch(scoopNum)
    {
        case SCOOP_NUM::SCOOP_1:
        scoop.writeMicroseconds(scoop1Up);
        break;
        case SCOOP_NUM::SCOOP_2:
        scoop.writeMicroseconds(scoop2Up);
        break;
        case SCOOP_NUM::SCOOP_3:
        scoop.writeMicroseconds(scoop3Up);
        break;
        case SCOOP_NUM::SCOOP_4:
        scoop.writeMicroseconds(scoop4Up);
        break;
    }
}

void scoop_level(Servo& scoop, SCOOP_NUM scoopNum)
{
    switch(scoopNum)
    {
        case SCOOP_NUM::SCOOP_1:
        scoop.writeMicroseconds(scoop1Level);
        break;
        case SCOOP_NUM::SCOOP_2:
        scoop.writeMicroseconds(scoop2Level);
        break;
        case SCOOP_NUM::SCOOP_3:
        scoop.writeMicroseconds(scoop3Level);
        break;
        case SCOOP_NUM::SCOOP_4:
        scoop.writeMicroseconds(scoop4Level);
        break;
    }
}

void scoop_adjust(Servo& scoop, const int degrees, SCOOP_NUM scoopNum)
{
    switch (scoopNum)
    {
        case SCOOP_NUM::SCOOP_1:
            if ((scoop.readMicroseconds() + (degrees*7.41)) >= scoop1Up){
                scoop.writeMicroseconds(scoop1Up);
            }
            else if ((scoop.readMicroseconds() + (degrees*7.41)) <= scoop1Down){
                scoop.writeMicroseconds(scoop1Down);
            }
            else{
                scoop.writeMicroseconds(scoop.readMicroseconds() + (degrees*7.41));
            }
            break;
        case SCOOP_NUM::SCOOP_2:
            if ((scoop.readMicroseconds() + (-1*(degrees*7.41))) <= scoop2Up){
                scoop.writeMicroseconds(scoop2Up);
            }
            else if ((scoop.readMicroseconds() + (-1*(degrees*7.41))) >= scoop2Down){
                scoop.writeMicroseconds(scoop2Down);
            }
            else{
                DEBUG_LOG("pulsewidth = %d", degrees);
                scoop.writeMicroseconds(scoop.readMicroseconds() + (-1*(degrees*7.41)));
            }
            break;
        // TODO: test real values of scoops 3/4 up and scoops 3/4 down
        case SCOOP_NUM::SCOOP_3:
            if ((scoop.readMicroseconds() + (degrees*7.41)) >= scoop3Up){
                scoop.writeMicroseconds(scoop3Up);
            }
            else if ((scoop.readMicroseconds() + (degrees*7.41)) <= scoop3Down){
                scoop.writeMicroseconds(scoop3Down);
            }
            else{
                scoop.writeMicroseconds(scoop.readMicroseconds() + (degrees*7.41));
            }
            break;
        case SCOOP_NUM::SCOOP_4:
            if ((scoop.readMicroseconds() + (-1*(degrees*7.41))) <= scoop4Up){
                scoop.writeMicroseconds(scoop4Up);
            }
            else if ((scoop.readMicroseconds() + (-1*(degrees*7.41))) >= scoop4Down){
                scoop.writeMicroseconds(scoop4Down);
            }
            else{
                scoop.writeMicroseconds(scoop.readMicroseconds() + (-1*(degrees*7.41)));
            }
            break;
    }
}

} // namespace

String scoop_read(SCOOP_NUM which, const String& param)
{
    int scoop_num = static_cast<int>(which);
    return {};
}

String scoop_write(SCOOP_NUM which, ScoopMode mode, const String& param)
{
    int scoopNumber = static_cast<int>(which);

    switch (mode) {
    case ScoopMode::UP:
        scoop_up(scoops[scoopNumber], which);
        return {};
    case ScoopMode::DOWN:
        scoop_down(scoops[scoopNumber], which);
        return {};
    case ScoopMode::LEVEL:
        scoop_level(scoops[scoopNumber], which);
        return {};
    case ScoopMode::ADJUST:
        // param.toInt() works using return atol(buffer)
        scoop_adjust(scoops[scoopNumber], param.toInt(), which);
    }

    return {};
}

void scoop_init()
{
    for (int i = 0; i < scoops.size(); ++i) {
        scoops[i].attach(pins::SCOOP_PINS[i]);
    }

    // for (int i = 0; i < scoops.size(); ++i) {
    //     scoops[i].writeMicroseconds(scoopLevel);
    // }
    scoops[0].writeMicroseconds(scoop1Level);
    scoops[1].writeMicroseconds(scoop2Level);
    scoops[2].writeMicroseconds(scoop3Level);
    scoops[3].writeMicroseconds(scoop4Level);
}

} // namespace cmd