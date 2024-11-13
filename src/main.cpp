#include <Arduino.h>
#include "SFM3019.h"

void setup() {
  Serial.begin(38400);
  delay(1000);

  Wire.begin();

  Serial.print("\n\n");

  if (SFMInitialize() == SFM_OK)
    Serial.println(F("SFM INIT OK"));
  else 
    Serial.println(F("SFM INIT FAIL"));


  if (SFMStartContReadAir() == SFM_OK) 
    Serial.println(F("CONT READ AIR OK"));
  else
    Serial.println(F("CONT READ AIR FAIL"));
}

void loop() {
  SFMReadSensor(&FlowMeter.rawFlow);
  Serial.println(FlowMeter.rawFlow);

  delay(10);
}