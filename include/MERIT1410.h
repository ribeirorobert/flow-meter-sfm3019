 /**
 * @file MERIT.h
 * @author Robert R. Gomes
 * @brief Library to communicate with pressure sensor
 * @version 1.0
 * @date 04-07-2025
 * 
 * @copyright Copyright (c) 2025
 * 
*/
#ifndef MERIT1410_H
#define MERIT1410_H

#include "Arduino.h"
#include "stdint.h"
#include "MPX5010DP.h"


#define MERIT1410_PIN   A7
#define MERIT_PRESSURE_INDEX   150U

typedef struct {
  bool filter = true;
  bool meas = false;  

  uint16_t pressIndex = 0;
  float rawValue;

  float pressBuffer[MERIT_PRESSURE_INDEX];
  float pressSum = 0;

  float offset = 0;
  float pressure;
  float filteredPressure;
  float flow = 0;
  float flow2 = 0;
} MERIT1410_Typedef;

void MERIT1410_init(void);
void MERIT1410_update_pressure(void);
void MERIT1410_update_flow(float K);
void MERIT1410_set_offset(uint16_t);
float MERIT1410_filtered_pressure(void);
int16_t MERIT1410_get_offset(void);


extern MERIT1410_Typedef merit;


#endif