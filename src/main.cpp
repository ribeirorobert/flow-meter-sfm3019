#include <Arduino.h>
#include "SFM3019.h"
#include "Pid.h"

#define BLOWER_CONTROL_PIN    7


float tidalVolume = 0;
uint32_t previousMillis;


float ctrlRaw = 0, flowRef = 20, prevError = 0;
float pTerm = 0, iTerm = 0, dTerm = 0;
uint32_t timer, prevIterationTime;
float kp = 2.0, ki = 7.8, kd = 0.0;
//float kp = 3.8918, ki = 16.0769, kd = 0.2355;

uint32_t refTimer;
uint8_t flag = 0;

void read_cmds(void);
void read_flow_sensor(void);
void blower_set_speed(uint8_t);
void update_flow_ref(void);
void debug(void);
void flow_control(float, float);

PidController_t flowCtrl;

void setup() {
  Serial.begin(38400);
  Wire.begin();

  SFMInit();

  pinMode(BLOWER_CONTROL_PIN, OUTPUT);
  blower_set_speed(0);

  PidControllerInit(&flowCtrl);
  PidControllerSetGains(&flowCtrl, kp, ki, 0.0, 0.0, 0.8, 0.001);
  PidControllerSetLimits(&flowCtrl, 150, -150, 150, -150, 255, 0);
  PidControllerSetFFD(&flowCtrl, 80);
}

void loop() {

  if (millis() - timer >= 10) {
    timer = millis();

    read_cmds();
    read_flow_sensor();
    //update_flow_ref();

    //ctrlRaw = PidControllerUpdate(&flowCtrl, flowRef, FlowMeter.rawFlow);
    //flow_control(flowRef, FlowMeter.rawFlow);
    blower_set_speed((uint8_t)ctrlRaw);
    debug();
  }  
}

void read_cmds(void) {

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '1') flowRef += 5;
    if (c == '2') flowRef -= 5;

    if (c == '3') ctrlRaw += 10;
    if (c == '4') ctrlRaw -= 10;


    if (c == 'q') kp += 0.1;
    if (c == 'w') kp -= 0.1;
    if (c == 'a') ki += 0.05;
    if (c == 's') ki -= 0.05;
    if (c == 'z') kd += 0.05;
    if (c == 'x') kd -= 0.05;

    PidControllerSetKp(&flowCtrl, kp);
    PidControllerSetKi(&flowCtrl, ki);
    PidControllerSetKd(&flowCtrl, kd);
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
  }
}

void blower_set_speed(uint8_t pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  analogWrite(BLOWER_CONTROL_PIN, pwmValue);
}

void debug(void) {
  // Serial.print(ctrlRaw);
  // Serial.print(F(";"));
  // Serial.println(FlowMeter.rawFlow);

  // Serial.print(flowRef);
  // Serial.print('\t');
  Serial.print(FlowMeter.rawFlow);
  // Serial.print('\t');
  // Serial.print(kp);
  // Serial.print('\t');
  // Serial.print(ki);
  // Serial.print('\t');
  // Serial.print(kd);
  // Serial.print('\t');
  // Serial.print(ctrlRaw);
  Serial.print('\n');
}

void flow_control(float desiredFlow, float feedback) {
  float sampleTime = (millis() - prevIterationTime) * 0.001;

  /*error*/
  float error = desiredFlow - feedback;

  /*proportional*/
  pTerm = kp * error;

  /*integration*/
  if (ki != 0) {
    iTerm = iTerm + (ki * ((error+prevError)/2) * sampleTime);
  } else {
    iTerm = 0;
  }

  /*derivative*/
  if (kd != 0) {
    dTerm = kd * (error - prevError) / sampleTime;
  } else {
    dTerm = 0;
  }

  ctrlRaw = pTerm + iTerm + dTerm + desiredFlow;

  if (ctrlRaw > 255) ctrlRaw = 255;
  if (ctrlRaw < 0) ctrlRaw = 0;

  prevError = error;
  prevIterationTime = millis();
}

void update_flow_ref(void) {

  if (!flag && millis() - refTimer >= 5000) {
    flowRef = 30;
    flag = 1;
    refTimer = millis();
  } else if (flag && millis() - refTimer >= 5000) {
    flowRef = 50;
    flag = 0;
    refTimer = millis();
  }
}