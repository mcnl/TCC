#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

class GPS {

    private:
        int RXPin, TXPin;
        uint32_t GPSBaud;
        TinyGPSPlus tinyGPS;
        SoftwareSerial ss(int RXPin, int TXPin);
    public:

        GPS(int RX, int TX);

        //Date Information
        int day, month, year, hour, minute;
        //Location Information
        double lastLatitude, lastLongitude, latitude, longitude;


        //GPS Function
        bool calcularGps();

        void init();

};

#endif