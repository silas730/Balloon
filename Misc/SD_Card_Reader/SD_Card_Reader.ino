#include <SD.h>
#include <SPI.h>


File dataFile;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!SD.begin()){
    Serial.println(F("SD card reader initialization failed"));
  }
  
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (SD.exists("data.txt")){
    Serial.println(F("It works"));
  } else {
    Serial.println("It doesnt work!");
  }
  dataFile.close();
}

void loop() {
  // put your main code here, to run repeatedly:

}
