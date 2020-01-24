#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Si7021.h>
#include "hsc_ssc_i2c.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

/*lm335 temp setup*/
#define LM335TEMP_PIN A0 //A0-A5
#define LM335UNITS 1 //0 for Kelvin. 1 for Celsius. 2 for Fahrenheit
#define REF_V 5.02

/*GPS Stuff*/
//8 > TX; 7 > RX (can be changed)
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false

/*Adafruit temp and humidity sensor*/
Adafruit_BMP280 bmp;
Adafruit_Si7021 humSensor = Adafruit_Si7021();

/*Geiger Counter Variables*/
#define GEIGER1_PIN 2 
int geiger_ct = 0;
void count_geiger(void);

//Used for data logging timing
unsigned long time_start = 0;

File dataFile;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(GEIGER1_PIN), count_geiger, FALLING);//Interrupt for geiger counter, triggers in falling edge
  
  Serial.begin(9600);
  //Starts Temp/Pressure sensor
  if (!bmp.begin()) {
    Serial.println(F("Could not find temp/pressure sensor"));
    while(1);
  }
  //Starts humidity Sensor
  if (!humSensor.begin()) {
    Serial.println(F("Could not find si7021 Sensor"));
    while(1);
  }
  //Starts honeywell pressure sensor
  if(initHoneywell() == 4){
      Serial.println(F("No honeywell pressure sensor detected"));
      while(1);
  }
  //Setup for Adafruit temp/hum sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /*Mode*/
                  Adafruit_BMP280::SAMPLING_X2, /*Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4, /*Pressure oversampling*/
                  Adafruit_BMP280::FILTER_X8,  /* Filtering */
                  Adafruit_BMP280::STANDBY_MS_500); /*Standby time */

  if (!SD.begin()){
    Serial.println(F("SD card reader initialization failed"));
  }
                  
  Serial.println(F("Time (sec):\tClicks per secound:\t Internal Pressure (Pa)\tInternal Temperature (°C) \tRel Humidity:\tExternal Pressure (Pa): \t External Temp: \tLatitude: \tLongitude: \tAltitude(m) \tSpeed(knts) \tDirection: \tTime\n"));

  //Opens the data file and marks the start
  dataFile = SD.open("data.txt", FILE_WRITE);
  dataFile.println("\n--------------Start--------------");
  dataFile.flush();

  /*Starts the GPS*/
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //this mode cuts out a bunch of the more technical data, if we need more I can change it.
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); //Update internal data storage every 5 seconds

  //openFile();
  //dataFile = SD.open("data.txt", FILE_WRITE);
}


 //TODO add checks for each sensor to see if it is connected and working. Also replace checks above ^
void loop() {
  char c = GPS.read();
  if ((c) && (GPSECHO)) Serial.write(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  if(millis() - time_start > 5000){
    
    time_start = millis();
    
    /*Time increment*/
    int timeFromStart = millis()/1000;
    /*Gieger counter average over 5 seconds*/
    float geigerCount = avgGeiger();
    /*Internal pressure from bmp280 sensor*/
    float intPressure = bmpPressure();
    /*Internal temperature from bmp280 sensor in celsius*/
    float intTemp = bmpTemperature();
    /*relatve humidity*/
    float relHumidity = humSensor.readHumidity();
    /*External Pressure from honeywell sensor*/
    float extPressure = honeywellPressure();
    /*External temperature sensor from lm335 sensor.*/
    float extTemp = lm335Temp(LM335TEMP_PIN, LM335UNITS);

    /*Latitude in decimal degrees*/
    float latitude = GPS.latitudeDegrees;
    /*Longitude in decimal degrees*/
    float longitude = GPS.longitudeDegrees;
    /*Altitude in meters above sea level*/
    float altitude = GPS.altitude;
    /*Current speed over ground in knots*/
    float gpsSpeed = GPS.speed;
    /*Course in degrees from true north*/
    float angle = GPS.angle;

    /*GMT time from gps Hours:Minutes:Seconds*/
    String time = getTime();

    //Writes and prints the time from start
    Serial.print(timeFromStart);
    dataFile.print(timeFromStart);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the average geiger count over 5 seconds
    Serial.print(geigerCount);
    dataFile.print(geigerCount);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the internal temperature and pressure from bmp 280
    Serial.print(intPressure);
    dataFile.print(intPressure);
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    Serial.print(intTemp);
    dataFile.print(intTemp);
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints relative humidity
    Serial.print(relHumidity);
    dataFile.print(relHumidity);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the external Pressure
    Serial.print(extPressure);
    dataFile.print(extPressure);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints external temperature
    Serial.print(extTemp);
    dataFile.print(extTemp);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints latitude in decimal degrees
    Serial.print(latitude);
    dataFile.print(latitude);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints longitude in decimal degrees
    Serial.print(longitude);
    dataFile.print(longitude);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints altitude in meters
    Serial.print(altitude);
    dataFile.print(altitude);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints current speed over ground in knots 
    Serial.print(gpsSpeed);
    dataFile.print(gpsSpeed);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints course in degrees from true north
    Serial.print(angle);
    dataFile.print(angle);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    Serial.println();
    dataFile.println();

    //Writes and prints the time down to the second in GMT
    Serial.print(time);
    dataFile.print(time);

    Serial.println();
    dataFile.println();    

    geiger_ct = 0;
  }
 dataFile.flush();
}

/**
 * Gets the time  in GMT from the gps
 * @return GMT time as hour:minute:second
 */
String getTime(){
  //String time = GPS.hour + ":" + GPS.minute + ":" + GPS.seconds;
  //return time;
  return String(GPS.hour) + ":" + GPS.minute + ":" + GPS.seconds;
}
/* //Creates file with a new name
void openFile(){
  String fileName = "data.txt";
  unsigned int fileNumber = 1;
  while (SD.exists(fileName)){
  fileName = String("data") + fileNumber + ".txt";
  fileNumber++;
  }
  dataFile = SD.open(fileName, FILE_WRITE);
}
*/
/**
 * returns the average geiger counts over 5 seconds
 */
float avgGeiger(){
  return (float)geiger_ct/5.0;
}
 
void count_geiger(void){
  geiger_ct++;//Count up on falling edge 
}

/**
 * Gets pressure from BM280 sensor
 * @return  pressure in Pascals
 */
float bmpPressure(){
  return bmp.readPressure();
}

/**
 * Gets temperature from BMP280 sensor
 * @return Temperature in Celsius
 */
float bmpTemperature(){
  return bmp.readTemperature();
}

//Returns temperture from LM335 sensor.
//Use analog pin A0 - A5
//Units as follos:
//0 - Kelvin
//1 - Celsius
//2 - Fahrenheit

float lm335Temp(int pin, int units){
  int raw = analogRead(pin);
  float temp_k = (float)raw*REF_V*100/1023;
  float temp_c = temp_k - 273.15;
  float temp_f = temp_c*1.8 + 32;

  if(units == 0) return temp_k;
  if(units == 1) return temp_c;
  if(units == 2) return temp_f;
  
}//End of lm335Temp
