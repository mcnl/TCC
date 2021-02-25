// Watch video here: https://www.youtube.com/watch?v=T4yyK0G9Zgg

/* Connection pins:

Arduino     Current Sensor B43
A0                  VT
A1                  AT
+5V                 VIN
+5V                 VOUT
GND                 GND
*/


#include <Wire.h>

#define VT_PIN 0 // connect VT
#define AT_PIN 1// connect AT

#define ARDUINO_WORK_VOLTAGE 5.0

float totawatt = 0.0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Voltage (V) / Current (A)");
}

void loop()
{
  int vt_temp = analogRead(VT_PIN);
  int at_temp = analogRead(AT_PIN);
  double voltage = vt_temp * (ARDUINO_WORK_VOLTAGE / 1023.0) * 5;
  double current = at_temp * (ARDUINO_WORK_VOLTAGE / 1023.0);
  Serial.print("Analog0: "); Serial.print(vt_temp); Serial.print(" / "); Serial.print("Analog1: "); Serial.print(at_temp, 5);  Serial.print(" ----- "); 
  Serial.print(voltage, 5); Serial.print(" / "); Serial.println(current, 5);
  totawatt = totawatt+((voltage)*(current)/7200);
  Serial.print("Watts: "); Serial.println((voltage)*(current));
  Serial.print("Total Watts: "); Serial.println(totawatt,3);
  delay(500);
}
