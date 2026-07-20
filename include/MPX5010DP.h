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

#ifndef MPX5010DP_H_
#define MPX5010DP_H_

#include <Arduino.h>
#include "stdint.h"


#define MPX5010_PIN                 A8
#define MPX5010_PRESSURE_OFFSET     41U         //offset = 200mV (datasheet) -> 0.2*1023/5 = 41 bits
#define MPX5010_FACTOR_PRESSURE     0.110725F   //5V/1023 = 4.88mV/bit, sensitivity (datasheet) = 4.413 mV/mmH20, mmH2O to cmH2O -> divide by 10 
                                                //4.88mV/4.413/10 = 0.110725
#define MPX5010_PRESSURE_INDEX      10U

typedef struct {
  uint8_t  pin;                                   
  float    offset;
  float    rawPressure;
  float    filtPressure;
  float    flow;
  float    buffer[MPX5010_PRESSURE_INDEX];
  float    pressSum;
  uint8_t  pressIndex;
  bool     filterReady;
} MPX5010_Typedef;

void  MPX5010_init            (MPX5010_Typedef *dev, uint8_t pin);
void  MPX5010_update_pressure (MPX5010_Typedef *dev);
void  MPX5010_update_flow     (MPX5010_Typedef *dev, float K);
void  MPX5010_set_offset      (MPX5010_Typedef *dev, float value);
void  MPX5010_update_filter   (MPX5010_Typedef *dev);

#endif