#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  char filename[] = "flight_xxx.txt";

  myFile = SD.open("Flight10", FILE_WRITE);

  if (myFile) {
    myFile.println("test");
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}
