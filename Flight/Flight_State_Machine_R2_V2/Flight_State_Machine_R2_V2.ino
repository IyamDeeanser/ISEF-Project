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
float launchLimit = 12.0;   //in m/s/s
float burnoutLimit = 6.0;  //in m/s/s
float burnoutTimeLimit = 2.4; //seconds after launch
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
float gearRatio = 5.0;
float TVCOffset = 90.0;

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
File dataFile, settingsFile;
char datafilename[15], settingsfilename[20];
int filenum;
const int chipSelect = BUILTIN_SDCARD;
float accelZ, accelX, accelY;
float gyroZ, gyroX, gyroY;
int state = 1;
float dataSpacer = 1;
float counter = 0; 
bool logDelay = true;
bool SDCheck = false;
bool settingsWrite = true;

//gyro biasing
float gyrobiasX, gyrobiasY, gyrobiasZ;
bool checkBias = true;

void setup() 
{
  Serial.begin(115200); // don't forget to set the baud rate to this too

  //initialize components
  accelInitialize();
  oriInitialize();
  miscInitialize();
  dataInitialize();
  gyroDebias();

  //create the time and interrupt function
  //this will cause the function chosen to repeat at any time interval determined here
  Timer1.initialize(timeInterval * 1000000); //this is in microseconds
  Timer1.attachInterrupt(TVC);
}

//the interrupt TVC function 
void TVC() //timer loop function
{
  //always runs
  states();
  timeCount();
  sensorRead();
  dataLogging();
  lights();
  
  //launchpad state
  if(state == 1) {
    launchDetect();
    logDelay = true;
  }

  //TVC powered flight
  if (state == 2) {
    quaternions();
    PID();
  }

  //descent and recovery
  if (state == 3) {
    parachutes();
    shutDown();
    createSettings();
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
    dataFile.println("Accel Initialization Error");
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
  //set data rate
  bool status2;
  status2 = gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_64HZ);
  if (status < 0) {
    dataFile.println("Gyro Initialization Error");
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
  //SD Card Initialization
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while(1)
    {
      analogWrite(Buzzer, HIGH);
    }
    return;
  }
  Serial.println("initialization done.");

  //-----data file writing-----
  //filewriting names
  filenum = 1;
  while(true) 
  {
    sprintf(datafilename, "Flight_%d.csv", filenum);
    if(!SD.exists(datafilename)) {
      break;
    }
    else {
      filenum++;
    }
  }

   dataFile = SD.open(datafilename, FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (dataFile) {
    Serial.print("Writing to data file...");
    dataFile.print("gyroX, ");
    dataFile.print("gyroY, ");
    dataFile.print("gyroZ, ");
    dataFile.print("accelX, ");
    dataFile.print("accelY, ");
    dataFile.print("accelZ, ");
    dataFile.print("yaw, ");
    dataFile.print("pitch, ");
    dataFile.print("roll, ");
    dataFile.print("pitch output, ");
    dataFile.print("yaw output, ");
    dataFile.print("state, ");
    dataFile.println("time");

  // close the file:
    dataFile.close();
  }
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening data file");
    while(1)
    {
      analogWrite(Buzzer, HIGH);
    }
  }
}

//create and log all of the flight settings
void createSettings()
{
  if(settingsWrite)
  {
    //-----settings file writing-----
    //filewriting names
    sprintf(settingsfilename, "Settin_%d.csv", filenum);

    settingsFile = SD.open(settingsfilename, FILE_WRITE);
    
    // if the file opened okay, write to it:
    if (settingsFile) {
      Serial.print("Writing to settings file...");
      settingsFile.print("Flight Number:,");                    settingsFile.println(filenum);
      settingsFile.print("Kp Pitch:,");                         settingsFile.println(Kp);
      settingsFile.print("Ki Pitch:,");                         settingsFile.println(Ki);
      settingsFile.print("Kd Pitch:,");                         settingsFile.println(Kd);
      settingsFile.print("Kp Yaw:,");                           settingsFile.println(Kp);
      settingsFile.print("Ki Yaw:,");                           settingsFile.println(Ki);
      settingsFile.print("Kd Yaw:,");                           settingsFile.println(Kd);
      settingsFile.print("TVC Max X Limit:,");                  settingsFile.println(maxAmount);
      settingsFile.print("TVC Max Y Limit:,");                  settingsFile.println(maxAmount);
      settingsFile.print("TVC Min X Limit:,");                  settingsFile.println(minAmount);
      settingsFile.print("TVC Min Y Limit:,");                  settingsFile.println(minAmount);
      settingsFile.print("TVC Gear Ratio X,");                  settingsFile.println(gearRatio);
      settingsFile.print("TVC Gear Ratio Y,");                  settingsFile.println(gearRatio);
      settingsFile.print("TVC Offset X:,");                     settingsFile.println(TVCOffset);
      settingsFile.print("TVC Offset Y:,");                     settingsFile.println(TVCOffset);
      settingsFile.print("Setpoint:,");                         settingsFile.println(setpoint);
      settingsFile.print("Gyroscope Bias X:,");                 settingsFile.println(gyrobiasX);
      settingsFile.print("Gyroscope Bias Y:,");                 settingsFile.println(gyrobiasY);
      settingsFile.print("Gyroscope Bias Z:,");                 settingsFile.println(gyrobiasZ);
      settingsFile.print("Launch Threshold:, ");                settingsFile.println(launchLimit);
      settingsFile.print("Burnout Accelerometer Threshold:, "); settingsFile.println(burnoutLimit);
      settingsFile.print("Burnout Time Threshold:, ");          settingsFile.println(burnoutTimeLimit);
      settingsFile.print("Loop Speed:, ");                      settingsFile.println(timeInterval);
      settingsFile.print("Time of Launch:, ");                  settingsFile.println(timeAtLaunch);

    // close the file:
      settingsFile.close();
    }
    
    else {
      // if the file didn't open, print an error:
      Serial.println("error opening settings file");
    }
    settingsWrite = false;
  }
}

//find the constant angular velocity from 0
void gyroDebias()
{
  if(checkBias)
  {  
    //light signal
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);

    float debiasTime = millis(), specialTime;
    int loopcount = 0;
    while((specialTime - debiasTime) < 10000)
    {
      gyro.readSensor();
      gyrobiasX = gyrobiasX + gyro.getGyroX_rads();
      gyrobiasY = gyrobiasY + gyro.getGyroY_rads();
      gyrobiasZ = gyrobiasZ + gyro.getGyroZ_rads();
      //delay(10);
      loopcount++;
      specialTime = millis();
    }
    gyrobiasX = gyrobiasX/loopcount;
    gyrobiasY = gyrobiasY/loopcount;
    gyrobiasZ = gyrobiasZ/loopcount;
    checkBias = false;
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
  if ((totalTime - timeAtLaunch) >= burnoutTimeLimit && accelZ <= burnoutLimit && state == 2)
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
  wS[1] = gyro.getGyroY_rads() - gyrobiasY;
  wS[2] = gyro.getGyroX_rads() - gyrobiasX;
  wS[3] = gyro.getGyroZ_rads() - gyrobiasZ;
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
  pitchOutput = (pitchP + pitchI + pitchD) * gearRatio + TVCOffset;

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
  yawOutput = (yawP + yawI + yawD) * gearRatio + TVCOffset;
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
  gyroX = gyro.getGyroX_rads() - gyrobiasX;
  gyroY = gyro.getGyroY_rads() - gyrobiasY;
  gyroZ = gyro.getGyroZ_rads() - gyrobiasZ;
}

//datalogging **still needs filewriting code**
void dataLogging()
{
  if(logDelay)
  {
    if((totalTime-counter) > dataSpacer)
    {
      datalog = true;
      counter = totalTime;
    }
    else
    {
      datalog = false;
    }
  }
 
  //write data to the SD card
  dataFile = SD.open(datafilename, FILE_WRITE);

  if (dataFile && datalog) {
    dataFile.print(gyroX);            dataFile.print(", ");
    dataFile.print(gyroY);            dataFile.print(", ");
    dataFile.print(gyroZ);            dataFile.print(", ");
    dataFile.print(accelX);           dataFile.print(", ");
    dataFile.print(accelY);           dataFile.print(", ");
    dataFile.print(accelZ);           dataFile.print(", ");
    dataFile.print(yaw);              dataFile.print(", ");
    dataFile.print(pitch);            dataFile.print(", ");
    dataFile.print(roll);             dataFile.print(", ");
    dataFile.print(pitchOutput-90);   dataFile.print(", ");
    dataFile.print(yawOutput-90);     dataFile.print(", ");
    dataFile.print(state);            dataFile.print(", ");
    dataFile.println(totalTime);
    dataFile.close(); // close the file

    //do all of the printing again in serial
    Serial.print(gyrobiasX);        Serial.print(", ");
    Serial.print(gyrobiasY);        Serial.print(", ");
    Serial.print(gyrobiasZ);        Serial.print(", ");
    Serial.print(gyroX);            Serial.print(", ");
    Serial.print(gyroY);            Serial.print(", ");
    Serial.print(gyroZ);            Serial.print(", ");
    Serial.print(accelX);           Serial.print(", ");
    Serial.print(accelY);           Serial.print(", ");
    Serial.print(accelZ);           Serial.print(", ");
    Serial.print(yaw);              Serial.print(", ");
    Serial.print(pitch);            Serial.print(", ");
    Serial.print(roll);             Serial.print(", ");
    Serial.print(pitchOutput-90);   Serial.print(", ");
    Serial.print(yawOutput-90);     Serial.print(", ");
    Serial.print(state);            Serial.print(", ");
    Serial.println(totalTime);
  }
  
  else if(!datalog)
  {
    dataFile.close();
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening data file");
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
  //parachuteServo.detach(); why the hell would you do this ;-;
  pitchServo.detach();
  yawServo.detach();

}

//nothing
void loop() {
}
