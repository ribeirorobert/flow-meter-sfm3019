#include <Arduino.h>
#include "SFM3019.h"
#include "Pid.h"


#define BLOWER_CONTROL_PIN  7
uint8_t blowerSpeed = 0;
uint8_t pwmValue = 0;

uint8_t flowRef = 0;
float kp = 5, ki = 0.2;

float tidalVolume = 0;
unsigned long previousMillis;

void read_cmds(void);
void read_flow_sensor(void);
void blower_set_speed(void);
void debug(void);

PidController_t flowCtrl;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(1000);

  SFMInit();

  pinMode(BLOWER_CONTROL_PIN, OUTPUT);

  PidControllerInit(&flowCtrl);
  PidControllerSetGains(&flowCtrl, kp, ki, 0.0, 0.0, 0.8, 0.001);
  PidControllerSetLimits(&flowCtrl, 25, -25, 100, -100, 255, 0);
}

void loop() {
  read_cmds();
  read_flow_sensor();
  //blower_set_speed();
  debug();

  uint8_t pwm = PidControllerUpdate(&flowCtrl, flowRef, FlowMeter.rawFlow);
  analogWrite(BLOWER_CONTROL_PIN, pwm);
}

void read_cmds(void) {

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    // if (c == '0') blowerSpeed = 0;
    // if (c == '1') blowerSpeed += 10;
    // if (c == '2') blowerSpeed -= 10;

    if (c == '0') flowRef = 0;
    if (c == '1') flowRef += 10;
    if (c == '2') flowRef -= 10;

    if (c == '3') {
      kp += 1.0;
      PidControllerSetKp(&flowCtrl, kp);
    }

    if (c == '4') {
      kp -= 1.0;
      PidControllerSetKp(&flowCtrl, kp);
    }

    if (c == '5') {
      ki += 0.2;
      PidControllerSetKi(&flowCtrl, ki);
    }

    if (c == '6') {
      ki -= 0.2;
      PidControllerSetKi(&flowCtrl, ki);
    }
  }
}

void read_flow_sensor(void) {
  if (SFMReadSensor(&FlowMeter.rawFlow) == INITIALIZED) {
    
    if (abs(FlowMeter.rawFlow) > 0.5) {
      tidalVolume += ((FlowMeter.rawFlow / 60) * (millis() - previousMillis));
      previousMillis = millis();

      if (tidalVolume < 0) tidalVolume = 0;
    } else {
      tidalVolume = 0;
    }

    // Serial.print(FlowMeter.rawFlow);
    // Serial.print('\t');
    // Serial.print(tidalVolume);
    // Serial.print('\n');
  }
}

void blower_set_speed(void) {
  if (blowerSpeed > 100) blowerSpeed = 100;
  if (blowerSpeed < 0) blowerSpeed = 0;

  pwmValue = ((float)blowerSpeed * 255.0F) / 100.0F;

  analogWrite(BLOWER_CONTROL_PIN, pwmValue);
}

void debug(void) {
  Serial.print(FlowMeter.rawFlow);
  Serial.print('\t');
  Serial.print(flowRef);
  Serial.print('\t');
  Serial.print(kp);
  Serial.print('\t');
  Serial.print(ki);
  Serial.print('\n');
}