#include "gps.h"

#define GPS_SERIAL Serial1

void GPS::init() {
    GPS_SERIAL.begin(9600);
}

void GPS::update() {
    if (millis() - lastUpdate > updateInterval) {
        while (GPS_SERIAL.available() > 0) {
            gps.encode(GPS_SERIAL.read());
        }

        if (gps.location.isValid()) {
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            lastUpdate = millis();
        }
    }
}

double GPS::getLatitude() {
    return latitude;
}

double GPS::getLongitude() {
    return longitude;
}