//New and improved version by Timothy Cai after attending a uni C++ class
#include "BMI088.h"
#include <TimerOne.h> //hardware timing library
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <cstdio>

//State Change Variables 
bool accelBuffer, accelCheck;
bool datalog = true;
Bmi088Accel accel(Wire, 0x18);
float launchLimit = 15
.0;
long timeAtLaunch = 0;
double totalTime = 0;

//TVC Variables
#define timeInterval 0.01 //this is so you can easily change the loop time
const int BMI = 0x68;
Bmi088Gyro gyro(Wire, 0x68);
bool gyroBuffer; //This allows the gyro data to be stored temporarily until the next cycle
float wS[4]; //this is to store angular velocities
float wI[4]; //this is to hold the initial desired orientation
float q_norm; //this is the norm of the last quaternion used in normalization
float q[4]; //this is the current derivative
float i_q[4]; //orientation of earth relative to sensor/the starting quaternion

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

//Pitch PID 
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
int LED_G = 15;
int LED_B = 16;
int Buzzer = 10;

//File Writing Variables
File myFile;
const int chipSelect = BUILTIN_SDCARD;
float accelZ, accelX, accelY;
float gyroZ, gyroX, gyroY;
int state = 1;
float dataSpacer = 100;
float counter = 0; 
bool logDelay = true;
bool SDCheck = false;

//yo future Timothy reading this, pretty sure something is wrong with this; needs time lib
int number = random(1000000, 9999999);// set random number

void setup() {
  Serial.begin(115200); // don't forget to set the baud rate to this too

  //initialize components
  accelInitialize();
  oriInitialize();
  miscInitialize();
  dataInitialize();


  //create the time and interrupt function
  //this will cause the function chosen to repeat at any time interval determined here
  Timer1.initialize(timeInterval * 1000000); //this is in microseconds
  Timer1.attachInterrupt(TVC);
}

//the interrupt TVC function **need to assemble functions underneath flight states**
void TVC() //timer loop function
{
  //always runs
  states();
  timeCount();
  sensorRead();
  Serial.println(accelCheck);
  
  //launchpad state
  if(state == 1) {
    launchDetect();
    //LED Signal
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  }

  //TVC powered flight
  if (state == 2) {
    //LED Signal
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }

  //descent and recovery
  if (state == 3) {
    //LED Signal 
    shutDown();
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  }
}

//-----setup----
//accelerometer initialization
void accelInitialize()
{
  //accelerometer
  int status;
  status = accel.begin();
  if (status < 0) {
    myFile.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
}

//orientation initialization
void oriInitialize()
{
  //start the sensors
  int status;
  status = gyro.begin();
  if (status < 0) {
    myFile.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }

  //Set the initial quaternion's orientation
  pre_q[0] = 1;
  pre_q[1] = 0;
  pre_q[2] = 0;
  pre_q[3] = 0;
}

//misc. components eg. servos, lights
void miscInitialize()
{
  //Setup Servos
  pitchServo.attach(3); //the 3rd pin is the x axis servo
  yawServo.attach(4); //the 4th pin is the y axis servo
 
  //setup lights and sound
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(Buzzer, OUTPUT);
}

//initialize datalogging
void dataInitialize()
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  // open the file. change name one day, perhaps 
  myFile = SD.open("test.csv", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.csv...");
    myFile.print("gyroX, ");
    myFile.print("gyroY, ");
    myFile.print("gyroZ, ");
    myFile.println("last thing on the list idk");

  // close the file:
    myFile.close();
  } 
  
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.csv");
  }
}


//-----main flight-----
//state changer
void states()
{
  //state 1
  if (accelCheck == false)
  {
    state = 1;
  }

  //state 2
  if (accelCheck && state == 1)
  {
    datalog = true;
    logDelay = false;
    state = 2;
  }

  //state 3
  if ((totalTime - timeAtLaunch) >= 2.4 && accelZ <= 6 && state == 2)
  {
    state = 3;
  }
}
//orienation 
void quaternions()
{
  gyro.readSensor();
  //Get the current angular rate
  wS[0] = 0;
  wS[1] = gyro.getGyroY_rads();
  wS[2] = gyro.getGyroX_rads();
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
  pitch = (-asin(2 * pre_q[1] * pre_q[3] + 2 * pre_q[0] * pre_q[2]));
  roll = (atan2(2 * pre_q[2] * pre_q[3] - 2 * pre_q[0] * pre_q[1], 2 * pre_q[0] * pre_q[0] + 2 * pre_q[3] * pre_q[3] - 1));
  //**unroll servo angles**
  //convert from radians to degrees
  yaw = yaw * (180 / M_PI);
  pitch = pitch * (180 / M_PI);
  roll = roll * (180 / M_PI);
}

//control algorithm **NEEDS WORK** - double check the +-90°
void PID()
{
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
}

//launch detection **needs work** - how is there gonna be a 50ms delay when the loop is running at 10ms????
void launchDetect()
{  
  //Acceleration check
  if (accelZ > launchLimit) {
    delay(50);
    if (accelZ > launchLimit) {
      accelCheck = true;
      timeAtLaunch = totalTime;
    }
  }
  
  else {
    accelCheck = false;
    //code for low-hertz datalogging on the pad **TIME MUST BE FIXED*
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
}

//sensor data
void sensorRead()
{
  //data reading; start sensor
  accel.readSensor();
  accelZ = -accel.getAccelY_mss();
  accelX = accel.getAccelZ_mss();
  accelY = accel.getAccelX_mss();

  gyro.readSensor();
  gyroX = gyro.getGyroX_rads();
  gyroY = gyro.getGyroY_rads();
  gyroZ = gyro.getGyroZ_rads();
}

//datalogging
void dataLogging()
{
  //do all of the printing again in serial

  //write data to the SD card
  myFile = SD.open("test.csv", FILE_WRITE);
  if (myFile) {
    myFile.print(accelZ);
    myFile.print(", ");
    myFile.print(accelX);
    myFile.print(", ");
    myFile.print(accelY);
    myFile.print(", ");
    myFile.print(yaw);
    myFile.print(", ");
    myFile.print(pitch);
    myFile.print(", ");
    myFile.print(roll);
    myFile.print(", ");
    myFile.print(pitchOutput-90);
    myFile.print(", ");
    myFile.print(yawOutput-90);
    myFile.print(", ");
    myFile.print(state);
    myFile.print(", ");
    myFile.println(totalTime/1000);
    myFile.close(); // close the file
  }
  
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.csv");
  }
}

//parachutes
void parachutes()
{
  //arm parachute servo
  parachuteServo.attach(5);
  
  //ejection
  parachuteServo.write(160);
}

//lights
void lights()
{
  if (accelCheck) {
    //LED Signal
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
    
  }

  else {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
    totalTime = totalTime + 5;
  }
}

//time recording
void timeCount()
{
  //note: time at launch is recorded in launchDetect function
  //continually record the time
  totalTime = totalTime + timeInterval;
}


//shutting down the system
void shutDown()
{ 
  //tracking noises
  analogWrite(Buzzer, HIGH);
  //note: we do NOT stop datalogging
  //disarm servos
  parachuteServo.detach();
  pitchServo.detach();
  yawServo.detach();

}

//nothing
void loop() {
}
