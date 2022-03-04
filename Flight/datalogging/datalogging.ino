#include "BMI088.h"
#include <SD.h>
#include <SPI.h>

File myFile;
const int chipSelect = BUILTIN_SDCARD;
float gyroZ, gyroX, gyroY;
const int BMI = 0x68;
Bmi088Gyro gyro(Wire, 0x68);
int gyrostatus;
int status;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
   //while (!Serial) {
    //; // wait for serial port to connect.
  //}

  gyroInitialize();
  SDInitialize();  
}

void loop()
{
  gyroRead();
  dataWrite();
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
}

void SDInitialize()
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
    myFile.println("gyroZ, ");

  // close the file:
    myFile.close();
  } 
  
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.csv");
  }
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
  Serial.print(gyroX);
  Serial.print(", ");
  Serial.print(gyroY);
  Serial.print(", ");
  Serial.println(gyroZ);
 
  myFile = SD.open("test.csv", FILE_WRITE);
  if (myFile) {    
    myFile.print(gyroX);
    myFile.print(", ");   
    myFile.print(gyroY);
    myFile.print(", ");
    myFile.println(gyroZ);
    myFile.close(); // close the file
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.csv");
  }
  delay(100);
}
