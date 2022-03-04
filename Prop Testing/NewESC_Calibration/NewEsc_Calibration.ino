#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
int highpoint = 0;
int lowpoint = 180;

void setup() {
  // Attach the ESC on pin 29
  ESC.attach(29,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(highpoint);
  delay(3000);
  ESC.write(lowpoint);
}

void loop() {
}
