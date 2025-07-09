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


MPX5010_Typedef mpx;


/**
 * @brief init sensor
 * 
 * @param void
 * @return nothing
*/
void MPX5010_init(void) {
  pinMode(MPX5010_PIN, INPUT);

  float sum = 0;
  mpx.offset = 0;
  for (uint8_t i = 0; i < 10; i ++) {
    MPX5010_update_pressure();
    sum += mpx.pressure;
    delay(20);
  }
  
  MPX5010_set_offset((float)sum * 0.1);
}


/**
 * @brief read raw pressure
 * 
 * @param void
 * @return nothing
*/
void MPX5010_update_pressure(void) {
  mpx.rawValue = analogRead(MPX5010_PIN);
  mpx.pressure = (float)(((mpx.rawValue - (float)MPX5010_PRESSURE_OFFSET) 
                          * (float)MXP5010_FACTOR_PRESSURE) 
                          + mpx.offset);
}


/**
 * @brief update flow
 * 
 * @param void
 * @return void
*/
void MPX5010_update_flow(float K) {
  MPX5010_update_pressure();
  MPX5010_filtered_pressure();

  float dP = (float)(mpx.filteredPressure * 0.0980665); //cmH2O to Kpa

  mpx.flow = sqrt(abs(dP)) * 60.0;
  (dP > 0) ? mpx.flow *= K : mpx.flow *= -K;
}

/**
 * @brief get offset
 * 
 * @param void
 * @return offset
*/
int16_t MPX5010_get_offset(void) {
  return mpx.offset;
}


/**
 * @brief set offset
 * 
 * @param void
 * @return void
*/
void MPX5010_set_offset(float value) {
  mpx.offset += (-1) * value;
}


/**
 * @brief filtered pressure 
 * 
 * @param void
 * @return pressure
*/
float MPX5010_filtered_pressure(void) {
  mpx.pressSum -= mpx.pressBuffer[mpx.pressIndex];
  mpx.pressBuffer[mpx.pressIndex] = mpx.pressure;
  mpx.pressSum += mpx.pressBuffer[mpx.pressIndex];

  mpx.pressIndex ++;
  if (mpx.pressIndex >= MPX5010_PRESSURE_INDEX) {
    mpx.pressIndex = 0;
    mpx.filter = true;
  }

  if (mpx.filter) {
    mpx.filteredPressure = (float)(mpx.pressSum / (float)MPX5010_PRESSURE_INDEX);
    return mpx.filteredPressure;
  } else {
    return (float)mpx.pressure;
  }
}