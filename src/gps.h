#include <TinyGPSPlus.h>

class GPS {
    TinyGPSPlus gps;
    double latitude;
    double longitude;
    byte second;
    byte minute;
    byte hour;
    byte day;
    byte month;
    int year;
    unsigned long lastUpdate;
    unsigned long updateInterval;
    public:
        GPS(unsigned long updateInterval) {
            this->updateInterval = updateInterval;
        }
        void init();
        void update();
        double getLatitude();
        double getLongitude();
        char* getDateTime();
        unsigned long getLastUpdate();
};