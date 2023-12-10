#include <Arduino.h>
#include <../gps/gps_init.h>

void setup() {
  gps_setup();
}

void loop() {
  displayLocation();
}

