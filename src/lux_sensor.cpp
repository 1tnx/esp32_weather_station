#include "lux_sensor.h"

void LuxSensor::init() {
    veml.begin();

    while (!veml.begin()) {
        Serial.println("Lux sensor not found, retrying in 1 second");
        delay(1000);
  }
}

void LuxSensor::update() {
    if (millis() - lastUpdate > updateInterval) {
        lux = veml.readLux();
        lastUpdate = millis();
    }
}

double LuxSensor::getLux() {
    return lux;
}