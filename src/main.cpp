#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "MPX5010DP.h"
#include "MERIT1410.h"
#include "SFM3019.h"

float tidalVolume = 0;
unsigned long previousMillis;
unsigned long timer0, timer1, timer2;

float alpha = 0.8;
float pFiltered = 0;

float filter_3th(float);
float venturi_fit(float);
float polynomial_fit(float);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  SFMInit();
  MPX5010_init();
  MERIT1410_init();
}

void loop() {

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '0') alpha = 0;
    if (c == '1') alpha += 0.05;
    if (c == '2') alpha -= 0.05;
    if (c == '3') alpha += 0.1;
    if (c == '4') alpha -= 0.1;
    if (c == '5') alpha += 1.0;
    if (c == '6') alpha -= 1.0;

    if (c == 'c') {
      merit.offset += (float)((-1.0) * merit.filteredPressure);
    }

    Serial.flush();
  }

  if (SFMReadSensor() == INITIALIZED) {
    if (abs(FlowMeter.rawFlow) > 0.5) {
      tidalVolume += ((FlowMeter.rawFlow / 60) * (millis() - previousMillis));
      previousMillis = millis();

      if (tidalVolume < 0) tidalVolume = 0;
    } else {
      tidalVolume = 0;
    }

    // Serial.print(FlowMeter.rawFlow);
    // Serial.print('\t');
    // Serial.print(FlowMeter.rawTemp);
    // Serial.print('\n');
  }
  
  if (millis() - timer0 >= 1) {
    timer0 = millis();

    MERIT1410_update_pressure();
    //pFiltered = (0.992 * pFiltered) + (0.008 * merit.pressure);
  }

  if (millis() - timer1 >= 20) {
    timer1 = millis();

    // Serial.print(polynomial_fit(merit.filteredPressure));
    // Serial.print('\t');
    pFiltered = (0.9 * pFiltered) + (0.1 * FlowMeter.rawFlow);

    Serial.print(merit.filteredPressure);
    Serial.print('\t');
    Serial.print(pFiltered);
    Serial.print('\n');
  }

}


float venturi_fit(float dP_cmH2O) {
  float dP_kPa = dP_cmH2O * 0.0980665F;
  float abs_dP = fabsf(dP_kPa);

  float flow = alpha * sqrt(abs_dP) * 60.0F;

  return (dP_kPa >= 0) ? flow : -flow;
}

float polynomial_fit(float dP_cmH2O) {
  float dP_kPa = dP_cmH2O * 0.0980665F;
  float abs_dP = fabsf(dP_kPa);

  //float flow = 106.6831 * sqrt(abs_dP) + 25.4251 * abs_dP + 0.2924;
  float flow = 98.2496 * sqrt(abs_dP) + 36.3923 * abs_dP + -1.3175;

  return (dP_kPa >= 0) ? flow : -flow;
}


#define GAIN   3.450423889e+002
static float xv[6], yv[6];

float filter_3th(float input) {
  xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; 
  xv[3] = input / GAIN;
  yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; 
  yv[3] =   (xv[0] + xv[3]) + 3 * (xv[1] + xv[2])
                + (  0.5320753683 * yv[0]) + ( -1.9293556691 * yv[1])
                + (  2.3740947437 * yv[2]);
  return yv[3];
}
