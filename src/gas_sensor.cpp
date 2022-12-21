#include "gas_sensor.h"

void BME688::init() {
    // Initialize the BME688
    while (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME688 sensor, retrying in 1 second");
        delay(1000);
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void BME688::update() {
    if (millis() - lastUpdate > updateInterval) {
        if (! bme.performReading()) {
            Serial.println("Failed to perform reading");
            return;
        }
        temperature = bme.temperature;
        pressure = bme.pressure / 100.0F;
        humidity = bme.humidity;
        lastUpdate = millis();
    }
}

double BME688::getTemperature() {
    return temperature;
}

double BME688::getPressure() {
    return pressure;
}

double BME688::getHumidity() {
    return humidity;
}