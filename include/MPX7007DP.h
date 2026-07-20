 /**
 * @file MPX7007DP.h
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 15-04-2026
 * 
 * @copyright Copyright (c) 2026
 * 
*/

#ifndef MPX7007DP_H_
#define MPX7007DP_H_

#include <Arduino.h>
#include "stdint.h"

#define MPX7007_VOUT_AT_0KPA        2.5f      
#define MPX7007_SENS_V_PER_KPA      0.286f
#define MPX7007_KPA_TO_CMH2O        10.1972f
#define MPX7007_PRESSURE_INDEX      10U

typedef struct {      
  uint8_t  pin;     
  float    offset;
  float    rawPressure;
  float    filtPressure;
  float    flow;
  float    pressBuffer[MPX7007_PRESSURE_INDEX];
  float    pressSum;
  uint8_t  pressIndex;
  bool     filterReady;
} MPX7007_Typedef;

void  MPX7007_init            (MPX7007_Typedef *dev, uint8_t pin);
void  MPX7007_update_pressure (MPX7007_Typedef *dev);
void  MPX7007_update_flow     (MPX7007_Typedef *dev, float K);
void  MPX7007_set_offset      (MPX7007_Typedef *dev, float value);
void  MPX7007_update_filter   (MPX7007_Typedef *dev);

#endif