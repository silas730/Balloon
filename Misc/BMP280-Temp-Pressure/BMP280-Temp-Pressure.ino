#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(9600); 
  if (!bmp.begin()) {
  Serial.println(F("Could not find temp/pressure sensor"));
  while(1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /*Mode*/
                  Adafruit_BMP280::SAMPLING_X2, /*Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4, /*Pressure oversampling*/
                  Adafruit_BMP280::FILTER_X8,  /* Filtering */
                  Adafruit_BMP280::STANDBY_MS_500); /*Standby time */
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(bmpPressure());
  Serial.println(bmpTemperature());
  delay(2000);
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
