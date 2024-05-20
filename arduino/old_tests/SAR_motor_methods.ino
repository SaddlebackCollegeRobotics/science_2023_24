// RECOMMENDATION FOR STEPPER MOTOR: Tie M0 and M2 to GND, and M1 to 5V.
// digital out pins
int step0 = 24;
int step1 = 24;
int dir0 = 22;
int dir1 = 22;

// pwm pins
int dc_mot = 3;
int servo = 4;


// make lower to make stepper faster
int speed = 10;

void setup() {
  // put your setup code here, to run once:
  // pin numbers
  pinMode(dc_mot, OUTPUT);
  pinMode(servo, OUTPUT);

  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);




  // down = LOW up = HIGH (i think please check, may need to have OPPOSITE values, but i dont think so)
  digitalWrite(dir0, HIGH);
  digitalWrite(dir1, HIGH);





}

void loop() {



}

void move_stepper(int speed){
  digitalWrite(step0, HIGH);
  digitalWrite(step1, HIGH);
  delay(speed);
  digitalWrite(step0, LOW);
  digitalWrite(step1, LOW);
  delay(speed);


} 

void dc_motor(){
  digitalWrite(dc_mot, HIGH);

  // 1500 is neutral, 1200 is fastest one way, 1800 is fastest the other way
  delayMicroseconds(1700);
  digitalWrite(dc_mot, LOW);
  delay(40);
}

// for the servos, screw it in so that position 1 is pointing AWAY from the logo
void servo_position1(){
  digitalWrite(servo, HIGH);
  delayMicroseconds(500);
  digitalWrite(servo, LOW);
  delay(2000);


}
void servo_position2(){
  digitalWrite(servo, HIGH);
  delayMicroseconds(1166);
  digitalWrite(servo, LOW);
  delay(2000);
}

