 /**
 * @file MPX7007DP.cpp
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 15-04-2026
 * 
 * @copyright Copyright (c) 2026
 * 
*/

#include "MPX7007DP.h"


void MPX7007_init(MPX7007_Typedef *dev, uint8_t pin) {
  memset(dev, 0, sizeof(MPX7007_Typedef));
  dev->pin = pin;
  pinMode(pin, INPUT);

  float sum = 0.0f;
  for (uint8_t i = 0; i < 20; i ++) {
    MPX7007_update_pressure(dev);
    sum += dev->rawPressure;
    delay(20);
  }

  MPX7007_set_offset(dev, sum * 0.05f);
}


void MPX7007_update_pressure(MPX7007_Typedef *dev) {
  float voltage = analogRead(dev->pin) * 0.004887f;

  dev->rawPressure = ((voltage - MPX7007_VOUT_AT_0KPA) / MPX7007_SENS_V_PER_KPA) 
    * MPX7007_KPA_TO_CMH2O + dev->offset;

  MPX7007_update_filter(dev);
}


void MPX7007_update_flow(MPX7007_Typedef *dev, float K) {
  float dP = (float)(dev->filtPressure * 0.0980665f); //cmH2O to Kpa

  dev->flow = sqrt(fabs(dP)) * 60.0f;
  (dP > 0) ? dev->flow *= K : dev->flow *= -K;
}


void MPX7007_set_offset(MPX7007_Typedef *dev, float value) {
  dev->offset += -value;
}


void MPX7007_update_filter(MPX7007_Typedef *dev) {
  dev->pressSum -= dev->pressBuffer[dev->pressIndex];
  dev->pressBuffer[dev->pressIndex] = dev->rawPressure;
  dev->pressSum += dev->pressBuffer[dev->pressIndex];

  dev->pressIndex ++;
  if (dev->pressIndex >= MPX7007_PRESSURE_INDEX) {
    dev->pressIndex = 0;
    dev->filterReady = true;
  }

  if (dev->filterReady) {
    dev->filtPressure = dev->pressSum / (float)MPX7007_PRESSURE_INDEX;
  } else {
    dev->filtPressure = dev->rawPressure;
  }
}