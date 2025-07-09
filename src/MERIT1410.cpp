 /**
 * @file MERIT1410.cpp
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 03-06-2025
 * 
 * @copyright Copyright (c) 2025
 * 
*/

#include "MERIT1410.h"

MERIT1410_Typedef merit;


/**
 * @brief init sensor
 * 
 * @param void
 * @return nothing
*/
void MERIT1410_init(void) {
  pinMode(MERIT1410_PIN, INPUT);

  float sum = 0;
  merit.offset = 0;

  for (uint8_t i = 0; i < 10; i ++) {
    MERIT1410_update_pressure();
    sum += merit.pressure;
    delay(20);
  }

  merit.offset = (float)((-0.1) * sum);
}


/**
 * @brief read raw pressure
 * 
 * @param void
 * @return nothing
*/
void MERIT1410_update_pressure(void) {
  merit.rawValue = (float)(analogRead(MERIT1410_PIN) * 0.004887F);

  float psi = (float)((0.30 * ((merit.rawValue - 0.5) / 4.0)) - 0.15);
  merit.pressure = (float)((psi * 70.306957) + merit.offset); //psi to cmH2O

  MERIT1410_filtered_pressure();
}


/**
 * @brief update flow
 * 
 * @param void
 * @return void
*/
void MERIT1410_update_flow(float Kv) {
  MERIT1410_update_pressure();
  MERIT1410_filtered_pressure();

  float dP1 = (float)(merit.pressure * 0.0980665); //cmH2O to Kpa
  float flowA = sqrt(abs(dP1)) * 60.0;
  (dP1 > 0) ? flowA *= Kv : flowA *= -Kv;
  merit.flow = (0.8 * merit.flow) + (0.2 * flowA);

  // float dP2 = (float)(mpx.filteredPressure * 0.0980665); //cmH2O to Bar
  // float flowB = Kv * sqrt(abs(mpx.filteredPressure * dP2)) * 60; //0.000980665
  // (dP2 > 0) ? flowB *= Kv : flowB *= -Kv;
  // merit.flow2 = flowB;
  // merit.flow2 = (0.8 * merit.flow2) + (0.2 * flowB);
}


/**
 * @brief filtered pressure 
 * 
 * @param void
 * @return pressure
*/
float MERIT1410_filtered_pressure(void) {
  merit.pressSum -= merit.pressBuffer[merit.pressIndex];
  merit.pressBuffer[merit.pressIndex] = merit.pressure;
  merit.pressSum += merit.pressBuffer[merit.pressIndex];

  merit.pressIndex ++;
  if (merit.pressIndex >= MERIT_PRESSURE_INDEX) {
    merit.pressIndex = 0;
    merit.filter = true;
  }

  if (merit.filter) {
    merit.filteredPressure = (float)(merit.pressSum / (float)MERIT_PRESSURE_INDEX);
    return merit.filteredPressure;
  } else {
    return (float)merit.pressure;
  }
}