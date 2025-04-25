#include <Arduino.h>
#include "SFM3019.h"
#include "Pid.h"

#define BLOWER_CONTROL_PIN    7

float tidalVolume = 0;
uint32_t previousMillis;


float ctrlRaw = 0, flowRef = 0;
float pTerm = 0, iTerm = 0, dTerm = 0;
float kp = 2.0, ki = 7.8, kd = 0.0;
//float kp = 3.8918, ki = 16.0769, kd = 0.2355;
float prevError = 0;
uint32_t loopPeriod, prevIterationTime, refPeriod;


uint8_t refFlag = 0;
float currentCtrlSignal = 0;
float filteredFlow = 0;

void read_cmds(void);
void read_flow_sensor(void);
void blower_set_speed(uint8_t);
void debug(void);
void flow_control(float, float);
void update_flow_ref(void);
float limit_rate(float, float, float);

PidController_t flowCtrl;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(1000);

  SFMInit();

  pinMode(BLOWER_CONTROL_PIN, OUTPUT);
  analogWrite(BLOWER_CONTROL_PIN, 0);

  PidControllerInit(&flowCtrl);
  PidControllerSetGains(&flowCtrl, kp, ki, kd, 0.0, 0.8, 0.001);
  PidControllerSetLimits(&flowCtrl, 150, -150, 250, -250, 255, 0);
  PidControllerSetFFD(&flowCtrl, 80);
}

void loop() {

  if (millis() - loopPeriod >= 10) {
    loopPeriod = millis();

    read_cmds();
    read_flow_sensor();
    update_flow_ref();

    filteredFlow = (0.8 * filteredFlow) + (0.2 * FlowMeter.rawFlow);

    ctrlRaw = PidControllerUpdate(&flowCtrl, flowRef, filteredFlow);
    float ctrlLimited = limit_rate(currentCtrlSignal, ctrlRaw, 0.05);
    currentCtrlSignal = ctrlLimited;

    //flow_control(flowRef, FlowMeter.rawFlow);
    blower_set_speed((uint8_t)ctrlRaw);
    debug();
  }  
}


float limit_rate(float current, float target, float percent) {
  float delta = target - current;
  float maxDelta = percent * fabs(current); // usa valor absoluto

  if (delta > maxDelta) {
    return current + maxDelta;
  } else if (delta < -maxDelta) {
    return current - maxDelta;
  } else {
    return target;
  }
}

void read_cmds(void) {

  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '0') ctrlRaw = 0;
    if (c == '1') ctrlRaw = 255;

    if (c == 'q') kp += 0.1;
    if (c == 'w') kp -= 0.1;
    if (c == 'a') ki += 0.5;
    if (c == 's') ki -= 0.5;
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

  Serial.print(flowRef);
  Serial.print('\t');
  Serial.print(FlowMeter.rawFlow);
  Serial.print('\n');
  // Serial.print(flowCtrl.pTerm);
  // Serial.print('\t');
  // Serial.print(flowCtrl.iTerm);
  // Serial.print('\t');
  // Serial.print(flowCtrl.dTerm);
  // Serial.print('\t');
  // Serial.print(flowCtrl.output);
  // Serial.print('\n');
}

void flow_control(float desiredFlow, float feedback) {
  float sampleTime = (millis() - prevIterationTime) * 0.001;

  /*error*/
  float error = desiredFlow - feedback;

  /*proportional*/
  pTerm = kp * error;

  /*integration*/
  if (ki != 0) {
    iTerm = iTerm + (ki * error * sampleTime);
  } else {
    iTerm = 0;
  }

  /*derivative*/
  if (kd != 0) {
    dTerm = kd * (error - prevError) / sampleTime;
  } else {
    dTerm = 0;
  }

  ctrlRaw = pTerm + iTerm + dTerm;// + desiredFlow;

  if (ctrlRaw > 255) ctrlRaw = 255;
  if (ctrlRaw < 0) ctrlRaw = 0;

  prevError = error;
  prevIterationTime = millis();
}

void update_flow_ref(void) {

  if (!refFlag && millis() - refPeriod >= 3000) {
    flowRef = 0;
    refFlag = 1;
    refPeriod = millis();
  } else if (refFlag && millis() - refPeriod >= 3000) {
    flowRef = 25;
    refFlag = 0;
    refPeriod = millis();
  }
}