#include "BMI088.h"
#include <TimerOne.h> //hardware timing library
#include <SD.h>
#include <SPI.h>

File myFile;
#define timeInterval 0.01 //this is so you can easily change the loop time
const int chipSelect = BUILTIN_SDCARD;
float gyroZ, gyroX, gyroY;
const int BMI = 0x68;
Bmi088Gyro gyro(Wire, 0x68);
int gyrostatus;
bool gyroBuffer; //This allows the gyro data to be stored temporarily until the next cycle
float wS[4]; //this is to store angular velocities
float wI[4]; //this is to hold the initial desired orientation
float q_norm; //this is the norm of the last quaternion used in normalization
float q[4]; //this is the current derivative
float i_q[4]; //orientation of earth relative to sensor/the starting quaternion
int status;

//this section is for code outside the timer function, which must be volatile, because MCU sucks lol
volatile float currentTime, previousTime, elapsedTime;
volatile float yaw, pitch, roll; //Tait-Bryan Angle variables
volatile float pre_q[4]; //this is the last estimate of orientation

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
   //while (!Serial) {
    //; // wait for serial port to connect.
  //}

  gyroInitialize();
  SDInitialize();  

  //create the time and interrupt function
  //this will cause the function chosen to repeat at any time interval determined here
  Timer1.initialize(timeInterval * 1000000); //this is in microseconds
  Timer1.attachInterrupt(quaternionLog);
}

void quaternionLog()
{
  //gyroRead();
  quaternions();
  dataWrite();
}

void loop()
{
}

void gyroInitialize()
{
  //start the sensors
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

void SDInitialize()
{
  Serial.print("Printing...");
}

void gyroRead()
{
  gyro.readSensor();
  gyroX = gyro.getGyroY_rads();
  gyroY = gyro.getGyroZ_rads();
  gyroZ = gyro.getGyroX_rads();
}

void dataWrite()
{
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pre_q[0]);
  Serial.print(", ");
  Serial.print(pre_q[1]);
  Serial.print(", ");
  Serial.print(pre_q[2]);
  Serial.print(", ");
  Serial.println(pre_q[3]);

}

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
