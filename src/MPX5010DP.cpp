 /**
 * @file MPX5010DP.cpp
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 03-06-2025
 * 
 * @copyright Copyright (c) 2025
 * 
*/

#include "MPX5010DP.h"


void MPX5010_init(MPX5010_Typedef *dev, uint8_t pin) {
  memset(dev, 0, sizeof(MPX5010_Typedef));
  dev->pin = pin;
  pinMode(pin, INPUT);

  float sum = 0.0f;
  
  for (uint8_t i = 0; i < 20; i++) {
    MPX5010_update_pressure(dev);
    sum += dev->rawPressure;
    delay(20);
  }

  MPX5010_set_offset(dev, sum * 0.05f);
}


void MPX5010_update_pressure(MPX5010_Typedef *dev) {
  uint16_t adcValue = analogRead(dev->pin);

  dev->rawPressure = (float)(((adcValue - (float)MPX5010_PRESSURE_OFFSET) 
    * (float)MPX5010_FACTOR_PRESSURE) + dev->offset);

  MPX5010_update_filter(dev);
}


void MPX5010_update_flow(MPX5010_Typedef *dev, float K) {
  MPX5010_update_pressure(dev);

  float dP = (float)(dev->filtPressure * 0.0980665f); //cmH2O to Kpa

  dev->flow = sqrt(fabs(dP)) * 60.0f;
  (dP > 0) ? dev->flow *= K : dev->flow *= -K;
}


void MPX5010_set_offset(MPX5010_Typedef *dev, float value) {
  dev->offset += -value;
}


void MPX5010_update_filter(MPX5010_Typedef *dev) {
  dev->pressSum -= dev->buffer[dev->pressIndex];
  dev->buffer[dev->pressIndex] = dev->rawPressure;
  dev->pressSum += dev->buffer[dev->pressIndex];

  dev->pressIndex ++;
  if (dev->pressIndex >= MPX5010_PRESSURE_INDEX) {
    dev->pressIndex = 0;
    dev->filterReady = true;
  }

  if (dev->filterReady) {
    dev->filtPressure = dev->pressSum / (float)MPX5010_PRESSURE_INDEX;
  } else {
    dev->filtPressure = dev->rawPressure;
  }
}