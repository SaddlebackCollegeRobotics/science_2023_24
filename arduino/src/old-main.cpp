
// NOLINTBEGIN
#include <Adafruit_SCD30.h>   // CO2 sensors
#include <Adafruit_VL53L0X.h> // ToF sensors
#include <Servo.h>            // PWM module . Stepper module
#include <Stepper.h>          // Stepper Motor
#include <Wire.h>             // Serial Comm

#include "cmds/all.hpp"
#include <util/array.hpp>

//    I2C ADDRESSES
#define I2C_MUX_ADDRESS 0x70 // by default
#define CO2_ADDRESS 0x61     // by default
#define lox0_ADDRESS 0x52    // by default
#define lox1_ADDRESS 0X00    // To Be Set to 0x53

#define Stepper_DIR_PIN_LRL 11
#define Stepper_DIR_PIN_LRR 9
#define Stepper_DIR_PIN_Drill 13
// consts
const int scoopDown = 550;
const int scoopUp = 1550;

// Sensors enum
enum sensors
{
    CO2_1,
    CO2_2,
    CO2_3,
    CO2_4,
    CO2_5,
    CO2_6,
    CO2_7,
    CO2_8,
    TOF_1,
    TOF_2,
    STEP_1,
    STEP_2,
    SERVO_1,
    SERVO_2,
    SERVO_3,
    SERVO_4,
    PUMP_1,
    PUMP_2,
    PUMP_3,
    PUMP_4,
    PUMP_5,
    PUMP_6,
    PUMP_7,
    PUMP_8
};

// global variables
int pulseWidth[4];          // record the position of each scoop
uint8_t transmission_in[8]; // temporary template for incoming transmission

// For this stepper motor, the number of steps per revolution
int stepsPerRevolution = 2048;
int motSpeed = 5;
int dt = 500;

// For the button
int buttonPin = 2;
int motDir = 1;
int buttonValNew;
int buttonValOld = 1;

union arg_t
{
    uint32_t n;
    int i;
    float f;
};

// structs and namespaces
// struct command_set{
//   uint8_t chip;
//   uint8_t func;
//   uint8_t param;
//   uint8_t crc;
// };

// control objects
Adafruit_SCD30 scd30;
Adafruit_VL53L0X lox[2];
Servo scoop[4];
// TODO: Ask Kaveh when he's done testing
Stepper myStepper[3] = {Stepper(stepsPerRevolution, 13, 12), Stepper(stepsPerRevolution, 9, 8),
                        Stepper(stepsPerRevolution, 11, 10)};

/*
 *
 *   I2C MUX functions
 *
 */

void mux_set_channel(uint8_t ch)
{
    // assume Wire is already set up.
    // otherwise Wire.begin();
    Wire.beginTransmission(I2C_MUX_ADDRESS);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

void mux_reset()
{
    Wire.beginTransmission(I2C_MUX_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();
}

/*
 *
 *   CO2 sensors control
 *
 */

void scd30_setup()
{
    if (scd30.begin()) {
        scd30.startContinuousMeasurement();
    } else {
        // TODO: init failed
    }
}

void scd30_set_interval(uint8_t t)
{
    if (scd30.setMeasurementInterval(t)) {
        // successfully updated
    }
}

void scd30_set_altitude(uint8_t alt)
{
    if (scd30.setAltitudeOffset(alt)) {
        // successfully updated
    }
}

void scd30_set_temp(uint8_t temp)
{
    if (scd30.setTemperatureOffset(temp)) {
        // successfully updated
    }
}

void scd30_get_data()
{ // == code here is copied from sample, adjust if needed
    if (scd30.dataReady()) {
#ifdef DEBUG
        Serial.println("Data available!");

        if (!scd30.read()) {
            Serial.println("Error reading sensor data");
            return;
        }

        Serial.print("Temperature: ");
        Serial.print(scd30.temperature);
        Serial.println(" degrees C");

        Serial.print("Relative Humidity: ");
        Serial.print(scd30.relative_humidity);
        Serial.println(" %");

        Serial.print("CO2: ");
        Serial.print(scd30.CO2, 3);
        Serial.println(" ppm");
        Serial.println("");
#endif
    } else {
        DEBUG_LOG("No data for scd30 tempurature sensor");
    }
}

/*
 *
 *   ToF sensors control
 *
 */

void lox_setup_continuous(Adafruit_VL53L0X& lox, uint8_t addr = lox0_ADDRESS)
{
    if (lox.begin(addr)) {
        lox.startRangeContinuous();
    } else {
#ifdef DEBUG
        Serial.println("Failed to setup TOF sensor with " + String(addr, HEX));
#endif
    }
}

void lox_measure_once(Adafruit_VL53L0X& lox)
{
    if (lox.isRangeComplete()) {
#ifdef DEBUG
        Serial.print("Distance in mm: ");
#endif
        lox.readRange(); // need a container to take the data in mm, possibly
                         // float/double precision
    }
}
/*
 *
 *   Pumps Authored by Aaron
 *
 */

namespace pump {
// pump -> pin# TBD
const int pumpPinArr[8] = {22, 24, 26, 28, 30, 32, 34, 36};
unsigned long startMillis[8];
uint8_t pumpPeriod[8];
bool pumpsActive[8];
} // namespace pump

void pump_duration()
{
    for (int i = 0; i < 8; i++) {
        if (pump::pumpsActive[i] && ((millis() - pump::startMillis[i]) >= pump::pumpPeriod[i])) {
            digitalWrite(pump::pumpPinArr[i], LOW);
            pump::pumpsActive[i] = false;
        }
    }
#ifdef DEBUG
    Serial.print("Current ms - start ms: ");
    Serial.println(millis() - pump::startMillis[7]); // only prints debugging message for pump id = 0;
#endif
}

void pump_run(uint8_t p_id, uint32_t t)
{
    pump::startMillis[p_id] = millis();
    digitalWrite(pump::pumpPinArr[p_id], HIGH);
    pump::pumpPeriod[p_id] = t;
    pump::pumpsActive[p_id] = true;
}

void pump_stop(uint8_t p_id)
{
    digitalWrite(pump::pumpPinArr[p_id], LOW);
}
// SCOOP

void scoop_down(uint8_t s_id)
{
    pulseWidth[s_id] = scoopDown;
    scoop[s_id].writeMicroseconds(pulseWidth[s_id]);
}

void scoop_up(uint8_t s_id)
{
    pulseWidth[s_id] = scoopUp;
    scoop[s_id].writeMicroseconds(pulseWidth[s_id]);
}

void scoop_move_increment(uint8_t s_id, uint8_t step_unsigned)
{
    int step = map(step_unsigned, 0, 255, 500, 2500);
    pulseWidth[s_id] = step;
    pulseWidth[s_id] = constrain(pulseWidth[s_id], 500, 2500);
    scoop[s_id].writeMicroseconds(pulseWidth[s_id]);
    scoop[s_id].readMicroseconds()
}

/*
 *
 *   Stepper
 *
 */

void stepper_ops_button(uint8_t stepper_id)
{
    buttonValNew = digitalRead(buttonPin);
    if (buttonValOld == 1 && buttonValNew == 0) {
        motDir = motDir * (-1);
    }
    myStepper[stepper_id].step(motDir * 1);
    buttonValOld = buttonValNew;
}

void stepper_dir_GPIO(uint8_t stepper_id)
{ // this would be default control
    // buttonValNew = digitalRead(buttonPin);

    if (buttonValOld == 1 && buttonValNew == 0) {
        motDir = motDir * (-1);
    }

    myStepper[stepper_id].step(motDir * 1);
    buttonValOld = buttonValNew;
}
/*
 *
 *   Check Sum (CRC)
 *
 */

static uint8_t crc8(uint8_t* data, int len)
{ // notice: const was removed from the requirements
    /*
     *
     * CRC-8 formula from page 14 of SHT spec pdf
     *
     * Test data 0xBE, 0xEF should yield 0x92
     *
     * Initialization data 0xFF
     * Polynomial 0x31 (x8 + x5 +x4 +1)
     * Final XOR 0x00
     */

    const uint8_t POLYNOMIAL(0x31);
    uint8_t crc(0xFF);

    for (int j = len; j; --j) {
        crc ^= *data++;

        for (int i = 8; i; --i) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

bool serial_input_handle()
{
    if (Serial.available()) {
        for (int i = 6; i; i--) {
            if (!Serial.available()) {
                // incomplete transmission handle
            }
            transmission_in[6 - i] = Serial.read();
        }
    } else
        return false;
    return true;
}

// Switching between different sensors
void instruction_handle()
{
    uint8_t bytes_0 = transmission_in[0];
    uint8_t bytes_1 = transmission_in[1];
    uint8_t param = transmission_in[2];
    switch (bytes_0) {
    case CO2_1:
    case CO2_2:
    case CO2_3:
    case CO2_4:
    case CO2_5:
    case CO2_6:
    case CO2_7:
    case CO2_8:
        mux_set_channel(bytes_0);
        switch (bytes_1) {
        case 0:
        default:
            scd30_get_data();
            break;
        }
        break;
    case TOF_1: // id = 8
    case TOF_2: // id = 9
        lox_measure_once(lox[bytes_0 - 8]);
        break;
    case STEP_1: // id = 10
    case STEP_2: // id = 11
        stepper_ops_button(bytes_0 - 10);
        break;
    case SERVO_1: // id 12 thru 15
    case SERVO_2:
    case SERVO_3:
    case SERVO_4:
        switch (bytes_1) {
        case 0:
            scoop_up(bytes_0 - 12);
            break;
        case 1:
            scoop_down(bytes_0 - 12);
            break;
        case 2:
            scoop_move_increment(bytes_0 - 12, param);
            break;
        }
        break;
    case PUMP_1: // id 16-23
    case PUMP_2:
    case PUMP_3:
    case PUMP_4:
    case PUMP_5:
    case PUMP_6:
    case PUMP_7:
    case PUMP_8:
        switch (bytes_1) {
        case 0:
            arg_t val = {
                (unsigned int)((((unsigned int)transmission_in[3]) << 24) | (((unsigned int)transmission_in[4]) << 16) |
                               (((unsigned int)transmission_in[5]) << 8) | ((unsigned int)transmission_in[6]))};
            pump_run(bytes_0 - 16, val.n);
            break;
        case 1: // FIXME (please...)
        default:
            pump_stop(bytes_0 - 16);
            break;
        }
        break;
    default:
        break;
    };
    //
}
/*
 *
 *   Init
 *
 */

void setup()
{
    Serial.begin(9600); // serial port init, may change to 115200
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    // Adafruit SCD30 init x8
    for (int i = 8; i; i--) {
        mux_set_channel(i);
        scd30_setup();
    }

    // mux reset to default channel
    mux_reset();

    // Adafruit VL53l0X init x2
    lox_setup_continuous(lox[0]);
    lox_setup_continuous(lox[1], lox1_ADDRESS);

    // scoop object init to pins
    scoop[0].attach(2);
    scoop[1].attach(3);
    scoop[2].attach(4);
    scoop[3].attach(5);

    // scoop up
    for (auto iter : scoop)
        iter.writeMicroseconds(scoopUp);

    // set up pumps
    for (int pin : pump::pumpPinArr) {
        pinMode(pin, OUTPUT);
    }

    // pump state to false
    for (auto& iter : pump::pumpsActive) {
        iter = false;
    }

    // Steppers
    myStepper[0].setSpeed(motSpeed); //====> need to perform same ops on all steppers

    // button code
    pinMode(buttonPin, INPUT);
    digitalWrite(buttonPin, HIGH);
}

/*
 *
 *   The loop()
 *
 */

void loop()
{
    DEBUG_LOG("%d parsed integer. %s", 123, "hi");
    // put your main code here, to run repeatedly:

    pump_duration();
    // parse input
    if (!serial_input_handle()) {
        // CRC checksum did not passed, debug msg here
    }

    // execute instructions
    instruction_handle();
    // build switch to distribute incoming instructions
    //  i/o handle
}
#endif