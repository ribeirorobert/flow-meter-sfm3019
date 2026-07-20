 /**
 * @file MPX7002DP.h
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 15-04-2026
 * 
 * @copyright Copyright (c) 2026
 * 
*/

#ifndef MPX7002DP_H_
#define MPX7002DP_H_

#include "ADS1115.h"
#include "stdint.h"


#define MPX7002_VOUT_AT_0KPA      2.5f        
#define MPX7002_SENS_V_PER_KPA    1.0f
#define MPX7002_KPA_TO_CMH2O      10.1972f
#define MPX7002_PRESSURE_INDEX    180U


typedef enum {
  MPX7002_CHANNEL_0 = 0,
  MPX7002_CHANNEL_1 = 1,
} MPX7002_Channel_t;

typedef struct {
  uint8_t  pin;                                  
  float    offset;
  float    rawPressure;
  float    filtPressure;
  float    flow;
  float    buffer[MPX7002_PRESSURE_INDEX];
  float    pressSum;
  uint8_t  pressIndex;
  bool     filterReady;
} MPX7002_Typedef;

void  MPX7002_init            (MPX7002_Typedef *dev, uint8_t pin);
void  MPX7002_update_pressure (MPX7002_Typedef *dev);
void  MPX7002_update_flow     (MPX7002_Typedef *dev, float K);
void  MPX7002_set_offset      (MPX7002_Typedef *dev, float value);
void  MPX7002_update_filter   (MPX7002_Typedef *dev);

#endif