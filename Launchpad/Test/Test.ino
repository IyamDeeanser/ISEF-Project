// the setup function runs once when you press reset or power the board
#include <Servo.h>
int buttonPin = 4;
int mosfetPin = 8;
int buttonState = 0;
Servo clamp1;
Servo clamp2;
Servo clamp3;
Servo clamp4;
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(mosfetPin, OUTPUT);
  clamp1.attach(3);
  clamp2.attach(5);
  clamp3.attach(6);
  clamp4.attach(9);
}

// the loop function runs over and over again forever
void loop() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    // turn LED on:
    for(int i = 7; i>-1; i--){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    }
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(mosfetPin, HIGH);
    delay(3000);
    clamp1.write(170);
    clamp2.write(170);
    clamp3.write(170);
    clamp4.write(170);
    delay(1000);
    clamp1.detach();
    clamp2.detach();
    clamp3.detach();
    clamp4.detach();
    digitalWrite(LED_BUILTIN, LOW);
  }
  else {
    // turn LED off:
    digitalWrite(LED_BUILTIN, LOW);
    clamp1.write(10);
    clamp2.write(10);
    clamp3.write(10);
    clamp4.write(10);
  }
}
