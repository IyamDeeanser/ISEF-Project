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
   ESC.write(175); //whatever speed
}

//arming sequence for the esc, decrease timing to 2000 ms for calibration
void armEsc()
{
  ESC.write(180);
  delay(4000);
  ESC.write(0);
  delay(1000);
}
