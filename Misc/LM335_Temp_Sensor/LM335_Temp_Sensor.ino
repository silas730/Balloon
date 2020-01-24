#define LM335TEMP_PIN A0
#define REF_V 5.02
int lm335_raw;
float lm335_cel;

float lm335Temp(int pin, int units);

void setup() {
  analogReference(DEFAULT);
  pinMode(LM335TEMP_PIN, INPUT);  
  Serial.begin(9600);
  
}

void loop() {
  lm335_raw = analogRead(LM335TEMP_PIN);
  
  lm335_cel = lm335Temp(LM335TEMP_PIN, 0);

  Serial.print("Temp (°C) = ");
  Serial.println(lm335_cel);

  delay(100);

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
