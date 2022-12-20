class Battery {
    int pin;
    int R1;
    int R2;
    int maxVoltage;
    int minVoltage;
    int voltage;
    int percentage;
    unsigned long lastUpdate;
    unsigned long updateInterval;

    public:
        Battery(int pin, int R1, int R2, int maxVoltage, int minVoltage, unsigned long interval) {
            this->pin = pin;
            this->R1 = R1;
            this->R2 = R2;
            this->maxVoltage = maxVoltage;
            this->minVoltage = minVoltage;
            updateInterval = interval;
        }
        void init();
        void update();
        int getVoltage();
        int getPercentage();
};