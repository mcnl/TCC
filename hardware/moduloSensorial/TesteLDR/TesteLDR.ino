#define Photoresistor 36 // for Arduino microcontroller
//#define Photoresistor 13 // for ESP8266 microcontroller
//#define Photoresistor 13 // for ESP32 microcontroller

void setup() {
  Serial.begin(9600);  // set baud rate to 9600
}

void loop() {
  int analog_value = analogRead(Photoresistor);
  int brightness = map(analog_value, 0, 4095, 0, 100);
  Serial.println(brightness);
  delay(100);
}