#include "Adafruit_VEML7700.h"

class LuxSensor {
    Adafruit_VEML7700 veml;
    double lux;
    unsigned long lastUpdate;
    unsigned long updateInterval;

    public:
        LuxSensor(unsigned long interval) {
            Adafruit_VEML7700 veml = Adafruit_VEML7700();
            updateInterval = interval;
        }
        void init();
        void update();
        double getLux();
};