#include <Servo.h>

String command;
String pitchDegreeStr;
String yawDegreeStr;
int pitchDegree;
int yawDegree;
int pitchMicroseconds;
int yawMicroseconds;
int comma;
int pitchPulsewidth;
int yawPulsewidth;

namespace camera{
  Servo pitch;
  Servo yaw;
  const int pitchPin = 5;
  const int yawPin = 6;
}

void setup() {
  Serial.begin(9600);

  camera::pitch.attach(camera::pitchPin, 500, 2500);
  camera::yaw.attach(camera::yawPin, 500, 2500);
  pinMode(camera::pitchPin, OUTPUT);
  pinMode(camera::yawPin, OUTPUT);

  camera::pitch.write(85);
  camera::yaw.write(85);


}

void loop() {
  command = "";
  pitchDegreeStr = "";
  yawDegreeStr = "";

  // CHECK BUFFER
  if (Serial.available() > 0){
    command = Serial.readString();

    // PARSE STRING
    comma = command.indexOf(',');
    pitchDegreeStr = command.substring(0, comma);
    yawDegreeStr = command.substring(comma + 1);
    yawDegree = yawDegreeStr.toInt();
    pitchDegree = pitchDegreeStr.toInt();

    if (pitchDegree < 50){
      pitchPulsewidth = 1056;
    }
    else{
      pitchPulsewidth = ((2000/180.0)*(pitchDegree - 5)) + 500;
    }
    yawPulsewidth = ((2000/180.0)*(yawDegree - 5)) + 500;

    Serial.print("Pitch: ");
    Serial.println(pitchPulsewidth);
    Serial.print("Yaw: ");
    Serial.println(yawPulsewidth);

    camera::pitch.writeMicroseconds(pitchPulsewidth);
    camera::yaw.writeMicroseconds(yawPulsewidth);
  }
}
