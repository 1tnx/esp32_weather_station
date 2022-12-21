#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

class BME688 {
    Adafruit_BME680 bme;
    double temperature;
    double pressure;
    double humidity;
    unsigned long lastUpdate;
    unsigned long updateInterval;

    public:
        BME688(unsigned long interval) {
            updateInterval = interval;
        }
        void init();
        void update();
        double getTemperature();
        double getPressure();
        double getHumidity();
};