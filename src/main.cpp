#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include "gps.h"
#include "battery.h"
#include "lux_sensor.h"
#include "gas_sensor.h"

// GPS update interval
#define GPS_INTERVAL 5000  // 5 seconds

// Battery settings
#define BATTERY_PIN             A0
#define BATTERY_R1              100 * 1000
#define BATTERY_R2              10 * 1000
#define BATTERY_INTERVAL        1000 * 60 * 5 // 5 minutes
#define BATTERY_MIN_VOLTAGE     3.7
#define BATTERY_MAX_VOLTAGE     4.2

// RFM95W pins
#define PIN_LMIC_NSS      8
#define PIN_LMIC_RXTX     LMIC_UNUSED_PIN
#define PIN_LMIC_RST      LMIC_UNUSED_PIN
#define PIN_LMIC_DIO0     20
#define PIN_LMIC_DIO1     19
#define PIN_LMIC_DIO2     LMIC_UNUSED_PIN

// OLED settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

// sensor settings
#define SENSOR_INTERVAL 5000 // 5 second

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  30

// I2C pins
#define I2C_SDA 6
#define I2C_SCL 7

GPS gps(GPS_INTERVAL);
Battery battery(BATTERY_PIN, BATTERY_R1, BATTERY_R2, BATTERY_MAX_VOLTAGE, BATTERY_MIN_VOLTAGE, BATTERY_INTERVAL);
BME688 gas(SENSOR_INTERVAL);
LuxSensor lux(SENSOR_INTERVAL);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// This be in little endian format
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0xAD, 0x8C, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format
static const u1_t PROGMEM APPKEY[16] = {0xDD, 0x85, 0xD2, 0x5E, 0xF2, 0x3D, 0x81, 0xFF, 0x03, 0xA2, 0x1E, 0x2B, 0x33, 0x8D, 0x63, 0xD6};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

#define PAYLOAD_SIZE     12

static osjob_t sendjob;
static osjob_t doWorkJob;

uint32_t doWorkIntervalSeconds = 3;

    // Schedule TX every this many seconds (might become longer due to duty
    // cycle limitations).
const unsigned TX_INTERVAL = 5;


const lmic_pinmap lmic_pins = {
    .nss = PIN_LMIC_NSS,
    .rxtx = PIN_LMIC_RXTX,
    .rst = PIN_LMIC_RST,
    .dio = { PIN_LMIC_DIO0, PIN_LMIC_DIO1, PIN_LMIC_DIO2 },
    .rxtx_rx_active = 0,
    .rssi_cal = 0,
    .spi_freq = 8000000,
};


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void oledDisplay(int size, int x, int y, float value, String unit)
{
    int charLen = 12;
    int xo = x + charLen * 3.2;
    int xunit = x + charLen * 3.6;
    int xval = x;
    display.setTextSize(size);
    display.setTextColor(WHITE);

    if (unit == "%")
    {
        display.setCursor(x, y);
        display.print(value, 0);
        display.print(unit);
    }
    else
    {
        if (value > 99)
        {
            xval = x;
        }
        else
        {
            xval = x + charLen;
        }
        display.setCursor(xval, y);
        display.print(value, 0);
        display.drawCircle(xo, y + 2, 2, WHITE); // print degree symbols (  )
        display.setCursor(xunit, y);
        display.print(unit);
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

        // update sensor data
        gps.update();
        gas.update();
        lux.update();
        battery.update();

        // read sensor data
        float temperature = gas.getTemperature();
        float humidity = gas.getHumidity();
        float pressure = gas.getPressure();
        float voltage = battery.getVoltage();
        float light = lux.getLux();
        double longitude = gps.getLongitude();
        double latitude = gps.getLatitude();
 
        uint8_t payload[PAYLOAD_SIZE] = { 0 };

        // Serial.print("Temperature: ");
        // Serial.println(temperature);
        // Serial.print("Humidity: ");
        // Serial.println(humidity);
        // Serial.print("Pressure: ");
        // Serial.println(pressure);
        // Serial.print("Voltage: ");
        // Serial.println(voltage);
        // Serial.print("Light: ");
        // Serial.println(light);
        // Serial.print("Latitude: ");
        // Serial.println(latitude);
        // Serial.print("Longitude: ");
        // Serial.println(longitude);
        
        // prepare payload
        payload[0] = (uint8_t)temperature;
        payload[1] = (uint8_t)((temperature - (int)temperature) * 10);
        payload[2] = (uint8_t)humidity;
        payload[3] = (uint8_t)pressure;
        payload[4] = (uint8_t)((pressure - (int)pressure) * 10);
        payload[5] = (uint8_t)voltage;
        payload[6] = (uint8_t)((voltage - (int)voltage) * 10);
        payload[7] = (uint8_t)light;
        payload[8] = (uint8_t)latitude;
        payload[9] = (uint8_t)((latitude - (int)latitude) * 10);
        payload[10] = (uint8_t)longitude;
        payload[11] = (uint8_t)((longitude - (int)longitude) * 10);

        for (int i = 0; i < PAYLOAD_SIZE; i++) {
            Serial.print(payload[i], HEX);
            Serial.print(", ");
        }
        Serial.println();

        oledDisplay(3,5,28,humidity,"%");
        oledDisplay(2,70,16,temperature,"C");
        display.display();

        LMIC_setTxData2(1, payload, PAYLOAD_SIZE, 0);
        Serial.println(F("Packet queued"));
        
        Serial.flush();
        display.clearDisplay();
        display.print("going to sleep");
        display.display();
        delay(1000);
        esp_deep_sleep_start();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void processWork(ostime_t doWorkJobTimeStamp)
{
    // This function is called from the doWorkCallback()
    // callback function when the doWork job is executed.

    // Uses globals: payloadBuffer and LMIC data structure.

    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWork if using OTAA and still joining.
    if (LMIC.devaddr == 0)
    {
        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.

        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
// TxRx is currently pending, do not send.
#ifdef USE_SERIAL
            printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
            printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
#endif
        }
        else
        {
            uint8_t payload[PAYLOAD_SIZE] = {0};

            // update sensor data
            gps.update();
            gas.update();
            lux.update();
            battery.update();

            // read sensor data
            float temperature = gas.getTemperature();
            float humidity = gas.getHumidity();
            float pressure = gas.getPressure();
            float voltage = battery.getVoltage();
            float light = lux.getLux();
            double longitude = gps.getLongitude();
            double latitude = gps.getLatitude();

            while (isnan(longitude) || isnan(latitude))
            {
                gps.update();
                longitude = gps.getLongitude();
                latitude = gps.getLatitude();
                delay(500);
            }
            

            payload[0] = (uint8_t)temperature;
            payload[1] = (uint8_t)((temperature - (int)temperature) * 10);
            payload[2] = (uint8_t)humidity;
            payload[3] = (uint8_t)pressure;
            payload[4] = (uint8_t)((pressure - (int)pressure) * 10);
            payload[5] = (uint8_t)voltage;
            payload[6] = (uint8_t)((voltage - (int)voltage) * 10);
            payload[7] = (uint8_t)light;
            payload[8] = (uint8_t)latitude;
            payload[9] = (uint8_t)((latitude - (int)latitude) * 10);
            payload[10] = (uint8_t)longitude;
            payload[11] = (uint8_t)((longitude - (int)longitude) * 10);

            for (int i = 0; i < PAYLOAD_SIZE; i++)
            {
                Serial.print(payload[i], HEX);
                Serial.print(", ");
            }
            Serial.println();

            LMIC_setTxData2(1, payload, PAYLOAD_SIZE, 0);
        }
    }
}

static void doWorkCallback(osjob_t *job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWork() which is called below.

    ostime_t timestamp = os_getTime();
#ifdef USE_SERIAL
    serial.println();
    printEvent(timestamp, "doWork job started", PrintTarget::Serial);
#endif

    // Do the work that needs to be performed.
    processWork(timestamp);

    // This job must explicitly reschedule itself for the next run.
    ostime_t startAt = timestamp + sec2osticks((int64_t)doWorkIntervalSeconds);
    os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);
}

void setup() {
    delay(5000);

    Wire.begin(I2C_SDA, I2C_SCL);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.display();

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    Serial.begin(9600);
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);
    // os_clearCallback(&doWorkJob);
    // os_setCallback(&doWorkJob, doWorkCallback);
    delay(1000); 

    gps.init();
	battery.init();
    gas.init();
    lux.init();

    delay(2000);
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
   os_runloop_once();
}