  int mode1 = 7;
  int mode2 = 8;
  int mode3 = 9;
  int step = 10;
  int dir = 11;
void setup() {
  // put your setup code here, to run once:
  // pin numbers


  pinMode(mode1, OUTPUT);
  pinMode(mode2, OUTPUT);
  pinMode(mode3, OUTPUT);
  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);

  // step size-- full step
  digitalWrite(mode1, LOW);
  digitalWrite(mode2, HIGH);
  digitalWrite(mode3, LOW);
  // down = LOW up = HIGH
  digitalWrite(dir, HIGH);


}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(step, HIGH);
  delay(1);
  digitalWrite(step, LOW);
  delay(1);
}
