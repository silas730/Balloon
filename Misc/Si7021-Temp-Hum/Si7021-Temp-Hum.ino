#include <Adafruit_Si7021.h>

Adafruit_Si7021 humSensor = Adafruit_Si7021();


void setup() {
  Serial.begin(9600);

  if (!humSensor.begin()) {
    Serial.println(F("Could not find si7021 Sensor"));
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(humSensor.readHumidity());
  delay(2000);
}
