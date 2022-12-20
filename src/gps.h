#include <TinyGPSPlus.h>

class GPS {
    TinyGPSPlus gps;
    double latitude;
    double longitude;
    unsigned long lastUpdate;
    unsigned long updateInterval;

    public:
        GPS(unsigned long interval) {
            updateInterval = interval;
        }
        void init();
        void update();
        double getLatitude();
        double getLongitude();
        unsigned long getLastUpdate();
};