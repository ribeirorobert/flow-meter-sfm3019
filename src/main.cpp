#include <Arduino.h>
#include "SFM3019.h"

float tidalVolume = 0;
unsigned long previousMillis;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(1000);

  SFMInit();
}

void loop() {
  if (SFMReadSensor(&FlowMeter.rawFlow) == INITIALIZED) {
    
    if (abs(FlowMeter.rawFlow) > 0.5) {
      tidalVolume += ((FlowMeter.rawFlow / 60) * (millis() - previousMillis));
      previousMillis = millis();

      if (tidalVolume < 0) tidalVolume = 0;
    } else {
      tidalVolume = 0;
    }

    Serial.print(FlowMeter.rawFlow);
    Serial.print('\t');
    Serial.print(tidalVolume);
    Serial.print('\n');
  }
}