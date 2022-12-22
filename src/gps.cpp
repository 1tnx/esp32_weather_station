#include "gps.h"

#define GPS_RX 18
#define GPS_TX 17

HardwareSerial serial(1);

void GPS::init() {
    serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

void GPS::update() {
    while (serial.available() > 0) {
        gps.encode(serial.read());
    }
    if (millis() - lastUpdate > updateInterval) {
        if (gps.location.isValid()) {
            lastUpdate = millis();
            latitude = gps.location.lat();
            longitude = gps.location.lng();
        }
        if (gps.date.isValid()) {
            day = gps.date.day();
            month = gps.date.month();
            year = gps.date.year();
        }
        if (gps.time.isValid()) {
            second = gps.time.second();
            minute = gps.time.minute();
            hour = gps.time.hour();
        }
    }
}

double GPS::getLatitude() {
    return latitude;
}

double GPS::getLongitude() {
    return longitude;
}

char* GPS::getDateTime() {
    char* dateTime = new char[23];
    sprintf(dateTime, "%02d/%02d/%02d %02d:%02d:%02d", day, month, year, hour, minute, second);
    return dateTime;
}

unsigned long GPS::getLastUpdate() {
    return lastUpdate;
}