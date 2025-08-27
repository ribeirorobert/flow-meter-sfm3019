#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "MPX5010DP.h"
#include "MERIT1410.h"
#include "SFM3019.h"

#define sensor_pin A4

float tidalVolume = 0;
unsigned long previousMillis;
unsigned long timer0, timer1, timer2;

float alpha = 10.0;
float flowFiltered = 0;

float voltage = 0;

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

  pinMode(sensor_pin, INPUT);
}

void loop() {

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '0') alpha = 0;
    if (c == '1') alpha += 0.1;
    if (c == '2') alpha -= 0.1;
    if (c == '3') alpha += 1.0;
    if (c == '4') alpha -= 1.0;
    if (c == '5') alpha += 5.0;
    if (c == '6') alpha -= 5.0;

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
    
    MPX5010_update_pressure();
    MPX5010_filtered_pressure();
  }

  if (millis() - timer1 >= 20) {
    timer1 = millis();

    flowFiltered = (0.9 * flowFiltered) + (0.1 * FlowMeter.rawFlow);
    voltage = (float)(analogRead(sensor_pin));

    float flow = (0.0004824 * pow(voltage, 2)) + (0.09084 * voltage) - 11.43;

    // Serial.print(merit.filteredPressure);
    // Serial.print('\t');
    // Serial.print(polynomial_fit(merit.filteredPressure));
    // Serial.print('\t');
    // Serial.print(flow);
    // Serial.print('\t');
    Serial.print(mpx.filteredPressure);
    Serial.print('\n');
  }

}


float venturi_fit(float dP_cmH2O) {
  float dP_kPa = dP_cmH2O * 0.0980665F;
  float abs_dP = fabsf(dP_kPa);

  float flow = alpha * sqrt(abs_dP) * 1.0F;

  return (dP_kPa >= 0) ? flow : -flow;
}

float polynomial_fit(float dP_cmH2O) {
  float dP_kPa = dP_cmH2O * 1; //0.0980665F;
  float abs_dP = fabsf(dP_kPa);

  // float flow = 106.6831 * sqrt(abs_dP) + 25.4251 * abs_dP + 0.2924;
  //float flow = 98.2496 * sqrt(abs_dP) + 36.3923 * abs_dP + -1.3175;
  float flow = 8.7488 * sqrt(abs_dP) + -0.0753 * abs_dP + 0.0914;

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
