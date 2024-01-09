#include <../gps/gps_init.h>
#include <Arduino.h>

void setup() {
  gps_setup();
}

void loop() {
  displayLocation();
}

