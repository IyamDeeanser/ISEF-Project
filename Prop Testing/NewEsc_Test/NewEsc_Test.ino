#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
int armingValue = 0;
int escvalue = 180;

void setup() {
  // Attach the ESC on pin 29
  ESC.attach(29,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(armingValue);
  delay(7000);
}

void loop() {
  ESC.write(escvalue);
}
