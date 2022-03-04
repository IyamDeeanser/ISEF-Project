#include <Servo.h>
Servo pitchServo;
Servo yawServo;
void setup() {
  // put your setup code here, to run once:
  pitchServo.attach(3); //the 3rd pin is the x axis servo
  yawServo.attach(4); //the 4th pin is the y axis servo
  pitchServo.write(90);
  yawServo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
}
