//servo lag test
#include <Servo.h>
Servo xServo;
int LEDpin = 16;
void setup() {
  // put your setup code here, to run once:
xServo.attach(3);
pinMode(LEDpin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(LEDpin, LOW);
xServo.write(125);//equivalent of 35 degrees
delay(2000);
xServo.write(90);
delay(2000);
digitalWrite(LEDpin, HIGH);
delay(2000);
}
