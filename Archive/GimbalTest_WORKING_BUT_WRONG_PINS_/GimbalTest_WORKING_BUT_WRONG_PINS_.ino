// Quaternion code by Timothy Cai (IyamDeeanser)
#include <BMI088.h>
#include <TimerOne.h> //hardware timing library
#include <Servo.h>
const int BMI=0x68;
int gyrostatus;
bool gyroBuffer; //This allows the gyro data to be stored temporarily until the next cycle
float wS[4]; //this is to store angular velocities
float wI[4]; //this is to hold the initial desired orientation
float q_norm; //this is the norm of the last quaternion used in normalization
float q[4]; //this is the current derivative
float i_q[4]; //orientation of earth relative to sensor/the starting quaternion
Servo xServo;
Servo yServo;
Bmi088Gyro gyro(Wire,0x68);
//this section is for code outside the timer function, which must be volatile, because MCU sucks lol
volatile float currentTime, previousTime, elapsedTime;
volatile float yaw, pitch, roll; //Tait-Bryan Angle variables
volatile float pre_q[4]; //this is the last estimate of orientation
volatile bool quaternionReady = 0; //this will be set to 1 when there's new data to be written, 0 by default

void setup() 
{
  int status;
  //USB Serial to print data
  Serial.begin(115200); // don't forget to set the baud rate to this too
  //start the sensors
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  xServo.attach(5);
  yServo.attach(29);
  //create the time and interrupt function
  //this will cause the function chosen to repeat at any time interval determined here
  Timer1.initialize(10000); //this is in microseconds
  Timer1.attachInterrupt(Orientation);
  //Set the initial quaternion's orientation
  pre_q[0] = 1;
  pre_q[1] = 0;
  pre_q[2] = 0;
  pre_q[3] = 0;
}
void Orientation() //timer loop function 
{
  //start sensor
  gyro.readSensor();
  //Get the current angular rate
  wS[0] = 0;
  wS[1] = gyro.getGyroX_rads();
  wS[2] = gyro.getGyroY_rads();
  wS[3] = gyro.getGyroZ_rads(); 
  //now we have to normalize the previous orientation to be used
  //First we have to find the norm of the last one
  q_norm = (sqrt(pre_q[0]*pre_q[0]+pre_q[1]*pre_q[1]+pre_q[2]*pre_q[2]+pre_q[3]*pre_q[3]));
  pre_q[0] = pre_q[0]/q_norm;
  pre_q[1] = pre_q[1]/q_norm;
  pre_q[2] = pre_q[2]/q_norm;
  pre_q[3] = pre_q[3]/q_norm;
  //Obtain the derivative of the next quaternion
   q[0]= 0.5*(pre_q[0]*wS[0]-pre_q[1]*wS[1]-pre_q[2]*wS[2]-pre_q[3]*wS[3]);
   q[1]= 0.5*(pre_q[0]*wS[1]+pre_q[1]*wS[0]+pre_q[2]*wS[3]-pre_q[3]*wS[2]);
   q[2]= 0.5*(pre_q[0]*wS[2]-pre_q[1]*wS[3]+pre_q[2]*wS[0]+pre_q[3]*wS[1]);
   q[3]= 0.5*(pre_q[0]*wS[3]+pre_q[1]*wS[2]-pre_q[2]*wS[1]+pre_q[3]*wS[0]);
   //calculate the "previous estimate" for the next cycle through integration, which is now the current quaternion ORIENTATION
   pre_q[0] = (pre_q[0] + q[0]*0.01); 
   pre_q[1] = (pre_q[1] + q[1]*0.01);
   pre_q[2] = (pre_q[2] + q[2]*0.01);
   pre_q[3] = (pre_q[3] + q[3]*0.01);
   //Convert to Euler, add condition for PID loop later
   yaw = (atan2(2*pre_q[1]*pre_q[2] - 2*pre_q[0]*pre_q[3], 2*pre_q[0]*pre_q[0] + 2*pre_q[1]*pre_q[1] - 1));
   pitch = (-asin(2*pre_q[1]*pre_q[3] + 2*pre_q[0]*pre_q[2]));
   roll = (atan2(2*pre_q[2]*pre_q[3] - 2*pre_q[0]*pre_q[1], 2*pre_q[0]*pre_q[0] + 2*pre_q[3]*pre_q[3] - 1));
   //convert from radians to degrees
   yaw = yaw*(180/M_PI);
   pitch = pitch*(180/M_PI);
   roll = roll*(180/M_PI);
   xServo.write(90+yaw);
   yServo.write(90+roll);
   //now to set this variable to 1, so it now knows it can print 
   quaternionReady = 1; //1 also means true, fyi
}
void loop() 
{
  if (quaternionReady){ //this also means if (quaternionReady = 1), but shorter
   //Print out resulting euler angles in serial
   quaternionReady = 0; //resets this variable for the next loop
  }
}
