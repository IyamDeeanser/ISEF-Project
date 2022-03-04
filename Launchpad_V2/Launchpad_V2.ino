#include <Servo.h>
Servo clamp1;
Servo clamp2;
Servo clamp3;
Servo clamp4;
int servoPin1 = 3;
int servoPin2 = 5;
int servoPin3 = 6;
int servoPin4 = 9;
int mosfetPin = 8;
int buttonPin = 4;
int LEDpin = 10;
int buttonState = 0;
void setup() {
// put your setup code here, to run once:
  //disonnected - state 0
  while(//bluetooth){
    delay(100);
    //check for bluetooth
  }
  //armed - state 1
  if(//bluetooth){
    //setup servos 
    clamp1.attach(servoPin1);
    clamp2.attach(servoPin2);
    clamp3.attach(servoPin3);
    clamp4.attach(servoPin4);
    //Button input
    pinMode(buttonPin, INPUT);
    //Mosfet stuff
    //digitalWrite(mosfetPin, LOW);
    //LED stuff
    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, HIGH); //lights up when loading
    //set starting position
    clamp1.write(10);
    clamp2.write(10);
    clamp3.write(10);
    clamp4.write(10);
    //wait 30 seconds to load
    delay(30000);
    digitalWrite(LEDpin, LOW); 
  }
  //countdown - state 2
  countDown();
}
void countDown(){
  //Launch Countdown
buttonState = digitalRead(buttonPin);
if (buttonState == LOW) {
    clamp1.write(45);
    digitalWrite(LED_BUILTIN, HIGH); 
    for(int i = 10; i>-1; i--){
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    }
    //digitalWrite(mosfetPin, HIGH); //fires the mosfet, rocket go boom
    //opens clamps
    clamp1.write(170);
    clamp2.write(170);
    clamp3.write(170);
    clamp4.write(170);
  }
else{
  delay(10);
  countDown();
  }
}
void loop() {
}
