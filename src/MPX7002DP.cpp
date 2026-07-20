 /**
 * @file MPX7002DP.cpp
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 15-04-2026
 * 
 * @copyright Copyright (c) 2026
 * 
*/

#include "MPX7002DP.h"

static float voltageToPressure(float voltage, float offset) {
  return (((voltage - MPX7002_VOUT_AT_0KPA) / MPX7002_SENS_V_PER_KPA) * MPX7002_KPA_TO_CMH2O) + offset;
}


void MPX7002_init(MPX7002_Typedef *dev, uint8_t pin) {
  memset(dev, 0, sizeof(MPX7002_Typedef));
  dev->pin = pin;
  pinMode(pin, INPUT);

  float sum = 0.0f;
  for (uint8_t i = 0; i < 20; i ++) {
    MPX7002_update_pressure(dev);
    sum += dev->rawPressure;
    delay(20);
  }

  MPX7002_set_offset(dev, sum * 0.05f);
}


void MPX7002_update_pressure(MPX7002_Typedef *dev) {
  float voltage = analogRead(dev->pin) * 0.004887f;

  dev->rawPressure = voltageToPressure(voltage, dev->offset);

  MPX7002_update_filter(dev);
}


void MPX7002_update_flow(MPX7002_Typedef *dev, float K) {
  float dP = (float)(dev->filtPressure * 0.0980665f); //cmH2O to Kpa

  dev->flow = sqrt(fabs(dP)) * 60.0f;
  (dP > 0) ? dev->flow *= K : dev->flow *= -K;
}


void MPX7002_set_offset(MPX7002_Typedef *dev, float value) {
  dev->offset += -value;
}


void MPX7002_update_filter(MPX7002_Typedef *dev) {
  dev->pressSum -= dev->buffer[dev->pressIndex];
  dev->buffer[dev->pressIndex] = dev->rawPressure;
  dev->pressSum += dev->buffer[dev->pressIndex];

  dev->pressIndex ++;
  if (dev->pressIndex >= MPX7002_PRESSURE_INDEX) {
    dev->pressIndex = 0;
    dev->filterReady = true;
  }

  if (dev->filterReady) {
    dev->filtPressure = dev->pressSum / (float)MPX7002_PRESSURE_INDEX;
  } else {
    dev->filtPressure = dev->rawPressure;
  }
}