// Quaternion code by Timothy Cai (IyamDeeanser)

#include <Wire.h>
#include <BMI088.h>

const int BMI=0x68;
int status;
bool gyroStatus;
float wS[4]; //this is to store angular velocities
float pre_q[4]; //this is the last estimate
float q[4] //this is the current estimate
float i_q[4] //orientation of earth relative to sensor/the starting quaternion

Bmi088Gyro gyro(Wire,0x68);

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  Serial.begin(115200);
   status = gyro.begin();
  //if (status < 0) {
    //code here to abort flight and write data to flash chip
  //Get the starting orientation
  wS[0] = 0
  wS[1] = gyro.getGyroX_rads();
  wS[2] = gyro.getGyroY_rads();
  wS[3] = gyro.getGyroZ_rads(); 
  //This is the code to obtain the derivative of the initial quaternion
  float i_q[4];
   i_q[0]= 0.5*(q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]);
   i_q[1]= 0.5*(q1[0]*q2[0]+q1[1]*q2[1]+q1[2]*q2[2]-q1[3]*q2[3]);
   i_q[2]= 0.5*(q1[0]*q2[0]-q1[1]*q2[1]+q1[2]*q2[2]+q1[3]*q2[3]);
   i_q[3]= 0.5*(q1[0]*q2[0]+q1[1]*q2[1]-q1[2]*q2[2]+q1[3]*q2[3]);
   //This is the code to obtain the orientation of the quaternion
   
}

void loop() {
  // put your main code here, to run repeatedly:

  //Store angular velocity components into a vector
  gyroStatus = gyro.readSensor();
  wS[0] = 0
  wS[1] = gyro.getGyroX_rads();
  wS[2] = gyro.getGyroY_rads();
  wS[3] = gyro.getGyroZ_rads();  
    

}
