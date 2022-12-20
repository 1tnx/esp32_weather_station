#include "Arduino.h"
#include "battery.h"

void Battery::init() {
    pinMode(pin, INPUT);
}

void Battery::update() {
    if (millis() - lastUpdate > updateInterval) {
        lastUpdate = millis();
        voltage = analogRead(pin);
        voltage = (voltage * (R1 + R2)) / R2;
        voltage = map(voltage, 0, 1023, 0, maxVoltage);
        percentage = map(voltage, minVoltage, maxVoltage, 0, 100);
    }
}

int Battery::getVoltage() {
    return voltage;
}

int Battery::getPercentage() {
    return percentage;
}