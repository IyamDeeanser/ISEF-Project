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
}

//the interrupt TVC function. It runs every 10ms, guaranteed, so there's a bit of code reorganizing to get the state change to work
void TVC() //timer loop function
{
}

void loop() {
}
