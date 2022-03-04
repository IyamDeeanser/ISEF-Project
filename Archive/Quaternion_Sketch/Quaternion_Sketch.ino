// Quaternion code by Timothy Cai (IyamDeeanser)

#include <Wire.h>
#include <BMI088.h>

const int BMI=0x68;
int gyrostatus;
bool gyroBuffer; //This allows the gyro data to be stored temporarily until the next cycle
float wS[4]; //this is to store angular velocities
float wI[4]; //this is to hold the initial desired orientation
float pre_q[4]; //this is the last estimate of orientation
float q_norm; //this is the norm of the last quaternion used in normalization
float q[4]; //this is the current derivative
float i_q[4]; //orientation of earth relative to sensor/the starting quaternion
float currentTime, previousTime, elapsedTime;
float yaw, pitch, roll; //Tait-Bryan Angle variables

Bmi088Gyro gyro(Wire,0x68);

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  Serial.begin(115200);
   gyrostatus = gyro.begin();
  //if (status < 0) {
    //code here to abort flight and write data to flash chip
  //Timing
  currentTime = millis();
  //Get the starting orientation
  wS[0] = 0;
  wS[1] = gyro.getGyroX_rads();
  wS[2] = gyro.getGyroY_rads();
  wS[3] = gyro.getGyroZ_rads(); 
  //Set the initial quaternion's orientation
  wI[0] = 1;
  wI[1] = 0;
  wI[2] = 0;
  wI[3] = 0;
  //This is the code to obtain the derivative of the initial quaternion
  float i_q[4];
   i_q[0]= 0.5*(wI[0]*wS[0]-wI[1]*wS[1]-wI[2]*wS[2]-wI[3]*wS[3]);
   i_q[1]= 0.5*(wI[0]*wS[0]+wI[1]*wS[1]+wI[2]*wS[2]-wI[3]*wS[3]);
   i_q[2]= 0.5*(wI[0]*wS[0]-wI[1]*wS[1]+wI[2]*wS[2]+wI[3]*wS[3]);
   i_q[3]= 0.5*(wI[0]*wS[0]+wI[1]*wS[1]-wI[2]*wS[2]+wI[3]*wS[3]);
   //This is the code to obtain the orientation of the initial quaternion, called pre_q because the usage of one variable for quaternion orientation is better  
   pre_q[0]= wI[0] + i_q[0]*currentTime;
   pre_q[1]= wI[1] + i_q[1]*currentTime;
   pre_q[2]= wI[2] + i_q[2]*currentTime;
   pre_q[3]= wI[3] + i_q[3]*currentTime;
}

void loop() {
  // put your main code here, to run repeatedly:
  //Timing
  previousTime = currentTime;
  previousTime = currentTime/1000;
  currentTime = millis();
  currentTime = currentTime/1000;
  elapsedTime = (currentTime - previousTime);
  //Get the current angular rate
  wS[0] = 0;
  wS[1] = gyro.getGyroX_rads();
  wS[2] = gyro.getGyroY_rads();
  wS[3] = gyro.getGyroZ_rads(); 
  //now we have to normalize the previous orientation to be used
  //First we have to find the norm of the last one
  q_norm = (sqrt((pow(pre_q[0],2)+pow(pre_q[1],2)+pow(pre_q[2],2)+pow(pre_q[3],2))));
  pre_q[0] = pre_q[0]/q_norm;
  pre_q[1] = pre_q[1]/q_norm;
  pre_q[2] = pre_q[2]/q_norm;
  pre_q[3] = pre_q[3]/q_norm;
  //Obtain the derivative of the next quaternion
   q[0]= 0.5*(pre_q[0]*wS[0]-pre_q[1]*wS[1]-pre_q[2]*wS[2]-pre_q[3]*wS[3]);
   q[1]= 0.5*(pre_q[0]*wS[0]+pre_q[1]*wS[1]+pre_q[2]*wS[2]-pre_q[3]*wS[3]);
   q[2]= 0.5*(pre_q[0]*wS[0]-pre_q[1]*wS[1]+pre_q[2]*wS[2]+pre_q[3]*wS[3]);
   q[3]= 0.5*(pre_q[0]*wS[0]+pre_q[1]*wS[1]-pre_q[2]*wS[2]+pre_q[3]*wS[3]);
   //This is the code to obtain the orientation of the next quaternion, which will be 
   //the "previous estimate" for the next cycle
   pre_q[0]= (pre_q[0] + q[0]*elapsedTime);
   pre_q[1]= (pre_q[1] + q[1]*elapsedTime);
   pre_q[2]=(pre_q[2] + q[2]*elapsedTime);
   pre_q[3]= (pre_q[3] + q[3]*elapsedTime);
   //Convert to Euler, add condition for PID loop later
   yaw = (atan2(2*pre_q[1]*pre_q[2] - 2*pre_q[0]*pre_q[3], 2*pre_q[0]*pre_q[0] + 2*pre_q[1]*pre_q[1] - 1));
   pitch = (-asin(2*pre_q[1]*pre_q[3] + 2*pre_q[0]*pre_q[2]));
   roll = (atan2(2*pre_q[2]*pre_q[3] - 2*pre_q[0]*pre_q[1], 2*pre_q[0]*pre_q[0] + 2*pre_q[3]*pre_q[3] - 1));
   //Print out on serialplotter
   Serial.print(yaw);
   Serial.print("\t");
   Serial.print(pitch);
   Serial.print("\t");
   Serial.print(roll);
   Serial.print("\n");
}
