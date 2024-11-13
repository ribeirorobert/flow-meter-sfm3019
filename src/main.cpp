#include <Arduino.h>
#include "SFM3019.h"

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(1000);

  SFMInit();
}

void loop() {
  if (SFMReadSensor(&FlowMeter.rawFlow) == INITIALIZED) {
    Serial.println(FlowMeter.rawFlow);
  }
}