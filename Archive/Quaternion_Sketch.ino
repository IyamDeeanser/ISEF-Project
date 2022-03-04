  // Quaternion code by Timothy Cai (IyamDeeanser)

#include <Wire.h>
#include <BMI088.h>

const int BMI=0x68;
int status;
bool gyroStatus;
float wX, wY, wZ, q0, q1, q2, q3;

Bmi088Gyro gyro(Wire,0x68);

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  Serial.begin(115200);
   status = gyro.begin();
  //if (status < 0) {
    //code here to abort flight and write data to flash chip
}

void loop() {
  // put your main code here, to run repeatedly:
  gyroStatus = gyro.readSensor();
  wX = gyro.getGyroX_rads();
  wY = gyro.getGyroY_rads();
  wZ = gyro.getGyroZ_rads();
    
    q0 = 
    q1 = 
    q2 = 
    q3 = 
}
