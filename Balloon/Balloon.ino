#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Si7021.h>
#include "hsc_ssc_i2c.h"
#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

boolean startupCheck = true;
int led = 31; //The LED pin

/*lm335 temp setup*/
#define LM335TEMP_PIN A0 //A0-A5
#define LM335UNITS 1 //0 for Kelvin. 1 for Celsius. 2 for Fahrenheit
#define REF_V 5.02

/*GPS Stuff*/
//8 > TX; 7 > RX (can be changed)
SoftwareSerial mySerial(11, 10);
Adafruit_GPS GPS(&mySerial);

/*Adafruit temp and humidity sensor*/
Adafruit_BMP280 bmp;
Adafruit_Si7021 humSensor = Adafruit_Si7021();

/*Geiger Counter Variables*/
#define GEIGER1_PIN 18 
int geiger_ct = 0;
void count_geiger(void);

//Used for data logging timing
unsigned long time_start = 0;

File dataFile;

void setup() {

  pinMode(led, OUTPUT);
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(GEIGER1_PIN), count_geiger, FALLING);//Interrupt for geiger counter, triggers in falling edge
  
  Serial.begin(9600);
  //Starts Temp/Pressure sensor
  if (!bmp.begin()) {
    Serial.println(F("Could not find temp/pressure sensor"));
    startupCheck = false;
    //while(1);
  }
  //Starts humidity Sensor
  if (!humSensor.begin()) {
    Serial.println(F("Could not find si7021 Sensor"));
    startupCheck = false;
    //while(1);
  }
  //Starts honeywell pressure sensor
  if(initHoneywell() == 4){
      Serial.println(F("No honeywell pressure sensor detected"));
      startupCheck = false;
      //while(1);
  }
  //Setup for Adafruit temp/hum sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /*Mode*/
                  Adafruit_BMP280::SAMPLING_X2, /*Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4, /*Pressure oversampling*/
                  Adafruit_BMP280::FILTER_X8,  /* Filtering */
                  Adafruit_BMP280::STANDBY_MS_500); /*Standby time */

  if (!SD.begin()){
    Serial.println(F("SD card reader initialization failed"));
    startupCheck = false;
  }
  
  if (lm335Temp(LM335TEMP_PIN, LM335UNITS) < -20 || lm335Temp(LM335TEMP_PIN, LM335UNITS) > 70) {
    Serial.println(F("Exteranl temperature sensor not found")); 
    startupCheck = false;
  }                
  Serial.println(F("Time(sec):\tClicks per secound:\tInternal Temperature(°C)\tRel Humidity:\tExternal Pressure(Pa):\tExternal Temp:\tLatitude:\tLongitude:\tAltitude(m)\tSpeed(knts)\tDirection:\tSatellites:\tTime\n"));

  //Opens the data file and marks the start
  dataFile = SD.open("data.txt", FILE_WRITE);
  dataFile.println("\n--------------Start--------------");
  dataFile.println(F("Time(sec):\tClicks per secound:\tInternal Temperature(°C)\tRel Humidity:\tExternal Pressure(Pa):\tExternal Temp:\tLatitude:\tLongitude:\tAltitude(m)\tSpeed(knts)\tDirection:\tSatellites:\tTime\n"));

  dataFile.flush();

  /*Starts the GPS*
  if (!GPS.begin(9600)){
    Serial.println(F("GPS failed to start"));
    startupCheck = false;
  }
  */
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //this mode cuts out a bunch of the more technical data, if we need more I can change it.
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); //Update internal data storage every 5 seconds

  
  
  //Turns the debug LED on. It will turn off when avgGeiger() is called if startupCheck is true and the geiger counter is working
  digitalWrite(led, HIGH);

  //openFile();
  //dataFile = SD.open("data.txt", FILE_WRITE);
}//End of setup


 //TODO add checks for each sensor to see if it is connected and working. Also replace checks above ^
void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  if(millis() - time_start > 5000){
    
    time_start = millis();
    
    /*Time increment*/
    unsigned long timeFromStart = millis()/1000;
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
    /*number of satalites in use*/
    int satellites = GPS.satellites;

    /*GMT time from gps Hours:Minutes:Seconds*/
    String time = getTime();

    //Writes and prints the time from start
    Serial.print(timeFromStart);
    dataFile.print(timeFromStart);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the average geiger count over 5 seconds
    Serial.print(geigerCount, 7);
    dataFile.print(geigerCount, 7);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the internal temperature and pressure from bmp 280
    Serial.print(intPressure);
    dataFile.print(intPressure);
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    Serial.print(intTemp, 7);
    dataFile.print(intTemp, 7);
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints relative humidity
    Serial.print(relHumidity, 7);
    dataFile.print(relHumidity, 7);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the external Pressure
    Serial.print(extPressure, 7);
    dataFile.print(extPressure, 7);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints external temperature
    Serial.print(extTemp, 7);
    dataFile.print(extTemp, 7);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints latitude in decimal degrees
    Serial.print(latitude, 7);
    dataFile.print(latitude, 7);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints longitude in decimal degrees
    Serial.print(longitude, 7);
    dataFile.print(longitude, 7);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints altitude in meters
    Serial.print(altitude, 7);
    dataFile.print(altitude, 7);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints current speed over ground in knots 
    Serial.print(gpsSpeed, 3);
    dataFile.print(gpsSpeed, 3);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));

    //Writes and prints course in degrees from true north
    Serial.print(angle, 3);
    dataFile.print(angle, 3);

    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Serial.println();
    //dataFile.println();

    //Writes and print the number of satellites in use
    Serial.print(satellites);
    dataFile.print(satellites);
    
    Serial.print(F("\t"));
    dataFile.print(F("\t"));
    
    //Writes and prints the time down to the second in GMT
    Serial.print(time);
    dataFile.print(time);
    
    Serial.println();
    dataFile.println();

   

    geiger_ct = 0;
  }
 dataFile.flush();
}//End of loop

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
  if (geiger_ct > 0 && startupCheck == true) {
    digitalWrite(led, LOW);
  }
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
