#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {

  // myservo.attach(13, 2050, 2400);  // attaches the servo on pin 9 to the servo object
  // myservo.write(90);

  // Go slightly above 180
  myservo.attach(13, 2050, 2600);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);

  delay(1000);

  // // Go to real 180
  myservo.attach(13, 2050, 2400);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);


  // Serial.begin(9600);  


}

void loop() {

  // myservo.write(180);
  // delay(1000);
  // myservo.write(90);
  // delay(1000);

  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   Serial.println(myservo.read());
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
  // for (pos = 360; pos >= -360; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   Serial.println(myservo.read());
  //   delay(15);                       // waits 15ms for the servo to reach the position
  
  // }

}