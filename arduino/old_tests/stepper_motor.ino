// POLOLU Stepper Motor Link:
// https://www.pololu.com/product/3096

#include <Stepper.h>
// For this stepper motor, the number of steps per revolution
int steps_per_revolution = 2048;
int motor_speed = 10;
int delay_time = 500;

// Adding Serial input
char received_direction = '\0';
int received_speed = 10;
bool new_data = true;
int motor_direction = 1;

enum pins : int {PIN_1 = 8, PIN_2, PIN_3, PIN_4};

Stepper myStepper(steps_per_revolution, 8, 10, 9, 11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
  myStepper.setSpeed(motor_speed);
}

// Serial Code
void getDir()
{
  if (Serial.available() > 0)
  {
    received_direction = Serial.read();
    new_data = true;
  }
}

void showDir()
{
  if (new_data == true)
  {
    Serial.print("Direction:\t");
    Serial.println(received_direction);
    new_data = false;
  }
}

void getSpeed()
{
  if (Serial.available() > 0)
  {
    Serial.println("HERE");
    received_speed = Serial.parseInt(SKIP_ALL, '\n');
    new_data = true;
  }
}

void showSpeed()
{
  if (new_data == true)
  {
    Serial.print("Speed:\t");
    Serial.println(received_speed);
    new_data = false;
  }
}

void loop() {

  // Unable to implement the direction at the same time as motor speed
  // getDir();
  // showDir();

  if (received_direction == 'f')
  {
    motor_direction = 1;
  }
  else if (received_direction == 'r')
  {
    motor_direction = -1;  
  }
  else if (received_direction == 's')
  {
    motor_direction = 0;
  }

  getSpeed();
  showSpeed();

  motor_speed = received_speed;
  myStepper.setSpeed(motor_speed);

  myStepper.step(motor_direction);
}
