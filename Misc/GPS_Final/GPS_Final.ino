#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
//8 > TX; 7 > RX (can be changed)
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO false

/*  
  float latitudeDegrees;    ///< Latitude in decimal degrees
  float longitudeDegrees;   ///< Longitude in decimal degrees
  float geoidheight;        ///< Diff between geoid height and WGS84 height
  float altitude;           ///< Altitude in meters above MSL
  float speed;              ///< Current speed over ground in knots
  float angle;              ///< Course in degrees from true north
  float magvariation;       ///< Magnetic variation in degrees (vs. true north)
  float HDOP;               ///< Horizontal Dilution of Precision - relative accuracy of horizontal position
  float VDOP;               ///< Vertical Dilution of Precision - relative accuracy of vertical position
  float PDOP;               ///< Position Dilution of Precision - Complex maths derives a simple, single number for each kind of DOP
  char lat;                 ///< N/S
  char lon;                 ///< E/W
  char mag;                 ///< Magnetic variation direction
  boolean fix;              ///< Have a fix?
  uint8_t fixquality;       ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
  uint8_t fixquality_3d;    ///< 3D fix quality (1, 3, 3 = Nofix, 2D fix, 3D fix)
  uint8_t satellites;       ///< Number of satellites in use


GPS.[any of the above methods]

  */


void setup() {
  Serial.begin(115200);
  delay(5000);

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //this mode cuts out a bunch of the more technical data, if we need more I can change it.
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); //Update internal data storage every 5 seconds
}

uint32_t timer = millis();

void loop() {
  char c = GPS.read();
  if ((c) && (GPSECHO)) Serial.write(c);
  if (GPS.newNMEArecieved()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  if (timer > millis()) timer = millis();
  if (millis() - timer > 2000) {
    timer = millis();
    if (GPS.fix) { //this will be run if the GPS has connection to a satellite

  //Serial.print(GPS.latitudeDegrees + ", " + GPS.longitudeDegrees);     for example

      
    } else { //even without satellite we can read internal clock and date


      
    }
  }

}
