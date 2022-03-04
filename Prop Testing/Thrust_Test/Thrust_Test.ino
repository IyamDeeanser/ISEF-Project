#include <Servo.h>

Servo ESC;     // create servo object to control the ESC

void setup() 
{
  // Attach the ESC on pin 9
  ESC.attach(29,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  armEsc();
}

void loop() 
{
   ESC.write(10); //stage 1
   delay(500);
   ESC.write(20); //stage 1
   delay(500);
   ESC.write(30); //stage 1
   delay(1000);
   ESC.write(60); //stage 2
   delay(1000);
   ESC.write(90); //stage 3
   delay(1000);
   ESC.write(120); //stage 4
   delay(1000);
   ESC.write(150); //stage 5
   delay(1000);
   ESC.write(180); //stage 6
}

//arming sequence for the esc, decrease timing to 2000 ms for calibration
void armEsc()
{
  ESC.write(180);
  delay(4000);
  ESC.write(0);
  delay(1000);
}
