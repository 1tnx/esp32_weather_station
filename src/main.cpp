#include <Arduino.h>
#include "gps.h"

#define INTERVAL 1000 * 3600 // 1 hour

GPS gps;

void setup() {
  gps.setUpdateInterval(INTERVAL);
  gps.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}