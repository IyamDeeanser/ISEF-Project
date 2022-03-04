#include "BMI088.h"
#include <TimerOne.h> //hardware timing library
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <cstdio>
//State Change Variables
bool accelBuffer;
bool accelCheck;
bool TVCstatus = false;
bool datalog = true;
Bmi088Accel accel(Wire, 0x18);
float launchLimit = 12.0;
long timeSinceLaunch = 0;
double totalTime = 0;
//TVC Variables
#define timeInterval 0.01 //this is so you can easily change the loop time
const int BMI = 0x68;
int gyrostatus;
bool gyroBuffer; //This allows the gyro data to be stored temporarily until the next cycle
float wS[4]; //this is to store angular velocities
float wI[4]; //this is to hold the initial desired orientation
float q_norm; //this is the norm of the last quaternion used in normalization
float q[4]; //this is the current derivative
float i_q[4]; //orientation of earth relative to sensor/the starting quaternion
Bmi088Gyro gyro(Wire, 0x68);
//this section is for code outside the timer function, which must be volatile, because MCU sucks lol
volatile float currentTime, previousTime, elapsedTime;
volatile float yaw, pitch, roll; //Tait-Bryan Angle variables
volatile float pre_q[4]; //this is the last estimate of orientation
//Servo Variables
Servo pitchServo;
Servo yawServo;
//PID VARIABLES
#define setpoint 0 //easier way of defining a word as a number
float Kp = 0.57;
float Ki = 0.0001;
float Kd = 0.39;
float maxAmount = 90 + 60;
float minAmount = 90 - 60;
//PitchPID Variables
float pitcherror;
float pitchlastError;
float pitchtotalError;
float pitchP;
float pitchI;
float pitchD;
float pitchOutput;

//Yaw PID
float yawerror;
float yawlastError;
float yawtotalError;
float yawP;
float yawI;
float yawD;
float yawOutput;

//Parachute Variables
Servo parachuteServo;

//Physical Variables
int LED_R = 14;
int LED_G = 16;
int LED_B = 15;
int Buzzer = 10;
//File Writing Variables
File data;
float accelZ, accelX, accelY;
float gyroZ, gyroX, gyroY;
int state = 1;
float dataSpacer = 100;
float counter = 0; 
bool logDelay = true;
bool SDCheck = false;
int number = random(1000000, 9999999);// set random number
void setup() {
  //USB Serial to print data
  Serial.begin(115200); // don't forget to set the baud rate to this too
  //accelerometer
  int status;
  status = accel.begin();
  if (status < 0) {
    data.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  //TVC Setup
  //start the sensors
  status = gyro.begin();
  if (status < 0) {
    data.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  //Setup Servos
  pitchServo.attach(3); //the 3rd pin is the x axis servo
  yawServo.attach(4); //the 4th pin is the y axis servo
  //create the time and interrupt function
  //this will cause the function chosen to repeat at any time interval determined here
  Timer1.initialize(timeInterval * 1000000); //this is in microseconds
  Timer1.attachInterrupt(TVC);
  //Set the initial quaternion's orientation
  pre_q[0] = 1;
  pre_q[1] = 0;
  pre_q[2] = 0;
  pre_q[3] = 0;
  //setup lights and sound
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  //setup datalogging
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  char s[10];
  sprintf(s,"File%d", number);
  data = SD.open(s, FILE_WRITE);
}

//the interrupt TVC function. It runs every 10ms, guaranteed, so there's a bit of code reorganizing to get the state change to work
void TVC() //timer loop function
{
  //data reading; start sensor
  accel.readSensor();
  accelZ = -accel.getAccelY_mss();
  accelX = accel.getAccelZ_mss();
  accelY = accel.getAccelX_mss();
  gyro.readSensor();
  gyroX = gyro.getGyroY_rads();
  gyroY = gyro.getGyroZ_rads();
  gyroZ = gyro.getGyroX_rads();
  //setup Parachutes
  parachuteServo.attach(5);
  //Acceleration check
  if (accelZ > launchLimit) {
    delay(50);
    if (accelZ > launchLimit) {
      accelCheck = true;
      datalog = true;
      logDelay = false;
    }
  }
  else {
    accelCheck = false;
    /*if (logDelay = true){
      if (counter == dataSpacer) {
        datalog = true;
        counter = 0;
      }
      else{
        datalog = false;
        counter = counter + 1;
        parachuteServo.write(90);
      }
    }*/
  }
  if (datalog == true) {
    data.print(accelZ);
    data.print(", ");
    data.print(accelX);
    data.print(", ");
    data.print(accelY);
    data.print(", ");
    data.print(gyroZ);
    data.print(", ");
    data.print(gyroX);
    data.print(", ");
    data.print(gyroY);
    data.print(", ");
    data.print(pitchOutput-90);
    data.print(", ");
    data.print(yawOutput-90);
    data.print(", ");
    data.print(state);
    data.print(", ");
    data.print(totalTime/1000);
    data.print("\n");
    /*Serial.print(accelZ);
    Serial.print(", ");
    Serial.print(accelX);
    Serial.print(", ");
    Serial.print(totalTime/1000);
    Serial.print("\n");*/ //doesn't work yet for some reason
  }
  if (accelCheck == true) {
    TVCstatus = true;
  }
  if (timeSinceLaunch >= 2400 && accelZ <= 6) {
    //stop TVC first thing
    TVCstatus = false;
    //ejection
    parachuteServo.write(160);
    state = 3;
    analogWrite(Buzzer, HIGH);
    delay(1000);
    //stop data
    //datalog = false;
    //detach servos
    parachuteServo.detach();
    pitchServo.detach();
    yawServo.detach();
    data.close();
  }
  if (TVCstatus == true) {
    //change state
    state = 2;
    datalog = true;
    //LED Signal
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    //Get the current angular rate
    wS[0] = 0;
    wS[1] = gyro.getGyroX_rads();
    wS[2] = gyro.getGyroY_rads();
    wS[3] = gyro.getGyroZ_rads();
    //now we have to normalize the previous orientation to be used
    //First we have to find the norm of the last one
    q_norm = (sqrt(pre_q[0] * pre_q[0] + pre_q[1] * pre_q[1] + pre_q[2] * pre_q[2] + pre_q[3] * pre_q[3]));
    pre_q[0] = pre_q[0] / q_norm;
    pre_q[1] = pre_q[1] / q_norm;
    pre_q[2] = pre_q[2] / q_norm;
    pre_q[3] = pre_q[3] / q_norm;
    //Obtain the derivative of the next quaternion
    q[0] = 0.5 * (pre_q[0] * wS[0] - pre_q[1] * wS[1] - pre_q[2] * wS[2] - pre_q[3] * wS[3]);
    q[1] = 0.5 * (pre_q[0] * wS[1] + pre_q[1] * wS[0] + pre_q[2] * wS[3] - pre_q[3] * wS[2]);
    q[2] = 0.5 * (pre_q[0] * wS[2] - pre_q[1] * wS[3] + pre_q[2] * wS[0] + pre_q[3] * wS[1]);
    q[3] = 0.5 * (pre_q[0] * wS[3] + pre_q[1] * wS[2] - pre_q[2] * wS[1] + pre_q[3] * wS[0]);
    //calculate the "previous estimate" for the next cycle through integration, which is now the current quaternion ORIENTATION
    pre_q[0] = (pre_q[0] + q[0] * timeInterval);
    pre_q[1] = (pre_q[1] + q[1] * timeInterval);
    pre_q[2] = (pre_q[2] + q[2] * timeInterval);
    pre_q[3] = (pre_q[3] + q[3] * timeInterval);
    //Convert to Euler, add condition for PID loop later
    yaw = (atan2(2 * pre_q[1] * pre_q[2] - 2 * pre_q[0] * pre_q[3], 2 * pre_q[0] * pre_q[0] + 2 * pre_q[1] * pre_q[1] - 1));
    roll = (-asin(2 * pre_q[1] * pre_q[3] + 2 * pre_q[0] * pre_q[2]));
    pitch = (atan2(2 * pre_q[2] * pre_q[3] - 2 * pre_q[0] * pre_q[1], 2 * pre_q[0] * pre_q[0] + 2 * pre_q[3] * pre_q[3] - 1));
    //unroll servo angles
    //convert from radians to degrees
    yaw = yaw * (180 / M_PI);
    roll = roll * (180 / M_PI);
    pitch = pitch * (180 / M_PI);
    //PID Time
    //pitchPID
    pitchlastError = pitcherror;
    pitcherror = setpoint - pitch;
    pitchtotalError = pitchtotalError + pitcherror * timeInterval;
    if (pitchtotalError > maxAmount) {
      pitchtotalError = maxAmount;
    }
    if (pitchtotalError < minAmount) {
      pitchtotalError = minAmount;
    }
    pitchP = Kp * pitcherror;   //Proportional part of pitchPID
    pitchI = Ki * pitchtotalError; //Integral part of pitchPID
    pitchD = Kd * (pitcherror - pitchlastError) / timeInterval; //Derivative part of pitchPID    The 0.01 is the hardware loop time
    //Now for the whole pitchPID equation
    pitchOutput = (pitchP + pitchI + pitchD) * 5 + 90;

    //yawPID
    yawlastError = yawerror;
    yawerror = setpoint - yaw;
    yawtotalError = yawtotalError + yawerror * timeInterval;
    if (yawtotalError > maxAmount) {
      yawtotalError = maxAmount;
    }
    if (yawtotalError < minAmount) {
      yawtotalError = minAmount;
    }
    yawP = Kp * yawerror;   //Proportional part of pitchPID
    yawI = Ki * yawtotalError; //Integral part of pitchPID
    yawD = Kd * (yawerror - yawlastError) / timeInterval; //Derivative part of pitchPID    The 0.01 is the hardware loop time
    //Now for the whole pitchPID equation
    yawOutput = (yawP + yawI + yawD) * 5 + 90;
    //Write outputs to servos
    pitchServo.write(pitchOutput);
    yawServo.write(yawOutput);
    timeSinceLaunch = timeSinceLaunch + 5;
    totalTime = totalTime + timeInterval*1000;
  }
  else {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    totalTime = totalTime + 5;
  }
}

void loop() {
}
