#include "GPS.h"

GPS::GPS(int TX, int RX){
    this->RXPin   = RX;
    this->TXPin   = TX;
    this->GPSBaud = 4800;
    this->ss      = SoftwareSerial(RX,TX);
    init();
}

void GPS::init() {
    Serial.begin(115200);
    ss.begin(GPSBaud);

    //DEBUG - DELETE LATER
    Serial.println(F("DeviceExample.ino"));
    Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
    Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();

}

bool GPS::calcularGps(){

  while (ss.available() > 0){
    if (tinyGPS.encode(ss.read())){
        Serial.print(F("Location: ")); 
        if (tinyGPS.location.isValid())
        {
            Serial.print(tinyGPS.location.lat(), 6);
            Serial.print(F(","));
            Serial.print(tinyGPS.location.lng(), 6);
        }
        else
        {
            Serial.print(F("INVALID"));
        }

        Serial.print(F("  Date/Time: "));
        if (tinyGPS.date.isValid())
        {
            Serial.print(tinyGPS.date.month());
            Serial.print(F("/"));
            Serial.print(tinyGPS.date.day());
            Serial.print(F("/"));
            Serial.print(tinyGPS.date.year());
        }
        else
        {
            Serial.print(F("INVALID"));
        }

        Serial.print(F(" "));
        if (tinyGPS.time.isValid())
        {
            if (tinyGPS.time.hour() < 10) Serial.print(F("0"));
            Serial.print(tinyGPS.time.hour());
            Serial.print(F(":"));
            if (tinyGPS.time.minute() < 10) Serial.print(F("0"));
            Serial.print(tinyGPS.time.minute());
            Serial.print(F(":"));
            if (tinyGPS.time.second() < 10) Serial.print(F("0"));
            Serial.print(tinyGPS.time.second());
            Serial.print(F("."));
            if (tinyGPS.time.centisecond() < 10) Serial.print(F("0"));
            Serial.print(tinyGPS.time.centisecond());
        }
        else
        {
            Serial.print(F("INVALID"));
        }

        Serial.println();
    }
    return true;
  } 

    if (millis() > 5000 && tinyGPS.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        Serial.println(tinyGPS.charsProcessed());
        delay(1000);
        return false;
    }     
}
