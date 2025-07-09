 /**
 * @file MPX5010DP.h
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 03-06-2025
 * 
 * @copyright Copyright (c) 2025
 * 
*/
#ifndef MPX5010DP_H
#define MPX5010DP_H

#include "Arduino.h"
#include "stdint.h"


#define MPX5010_PIN                 A8
#define MPX5010_PRESSURE_OFFSET     41U         //offset = 200mV (datasheet) -> 0.2*1023/5 = 41 bits
#define MXP5010_FACTOR_PRESSURE     0.110725F   //5V/1023 = 4.88mV/bit, sensitivity (datasheet) = 4.413 mV/mmH20, mmH2O to cmH2O -> divide by 10 
                                                //4.88mV/4.413/10 = 0.110725
#define MPX5010_PRESSURE_INDEX      10U

typedef struct {
  bool filter = true;
  bool meas = false;

  uint8_t pressIndex = 0;
  uint16_t rawValue;

  float pressBuffer[MPX5010_PRESSURE_INDEX];
  float pressSum = 0;

  float offset = 0;
  float pressure;
  float filteredPressure;
  float flow = 0;
} MPX5010_Typedef;


void MPX5010_init();
void MPX5010_update_pressure(void);
void MPX5010_update_flow(float K);
void MPX5010_set_offset(float);
float MPX5010_filtered_pressure(void);
int16_t MPX5010_get_offset(void);


extern MPX5010_Typedef mpx;

#endif