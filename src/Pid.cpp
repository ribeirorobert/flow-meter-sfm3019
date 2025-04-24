#include "Pid.h"


void PidControllerInit(PidController_t* pid) {
  pid->pTerm = 0;
  pid->iTerm = 0;
  pid->dTerm = 0;

  pid->error = 0;
  pid->prevError = 0;
  pid->satError = 0;

  pid->prevFeedback = 0;
  pid->feedForward = 0;

  pid->output = 0;
  pid->preSatOutput = 0;
  pid->prevOutput = 0;

  pid->kp = 0;
  pid->ki = 0;
  pid->kd = 0;
  pid->ks = 0;
  pid->ku = 0;
  pid->tau = 0;

  pid->lastTime = millis();
}


void PidControllerSetGains(PidController_t* pid, float Kp, float Ki, float Kd, float Ks, float Ku, float Tau) {
  pid->kp = Kp;
  pid->ki = Ki;
  pid->kd = Kd;
  pid->ks = Ks;
  pid->ku = Ku;

  pid->tau = Tau;
}


void PidControllerSetLimits(PidController_t* pid, float maxError, float minError, float maxInt, float minInt, float maxOut, float minOut) {
  pid->maxError = maxError;
  pid->minError = minError;

  pid->maxIntegr = maxInt;
  pid->minIntegr = minInt;

  pid->maxOutput = maxOut;
  pid->minOutput = minOut;
}


float PidControllerUpdate(PidController_t* pid, float reference, float feedback) {

  /*sample time*/
  pid->sampleTime = (millis() - pid->lastTime) * 0.001;

  /*error*/
  pid->error = reference - feedback;
  // if (pid->error > pid->maxError) pid->error = pid->maxError;
  // if (pid->error < pid->minError) pid->error = pid->minError;

  /*proportional term*/
  if (pid->kp != 0) {
    pid->pTerm = pid->kp * pid->error;
  } else {
    pid->pTerm = 0;
  }

  /*integral term*/
  if (pid->ki != 0) {
    pid->iTerm = pid->iTerm + (pid->ki * pid->error * pid->sampleTime);
    //pid->iTerm = pid->iTerm + (pid->ki * ((pid->error + pid->prevError)/2) * pid->sampleTime) + (pid->ku * pid->satError * pid->sampleTime);
    if (pid->iTerm > pid->maxIntegr) pid->iTerm = pid->maxIntegr;
    if (pid->iTerm < pid->minIntegr) pid->iTerm = pid->minIntegr;
  } else {
    pid->iTerm = 0;
  }

  /*derivative term*/
  if (pid->kd != 0) {
    pid->dTerm = ((2.0 * pid->kd * (pid->error- pid->prevError))
               + ((2.0 * pid->tau - pid->sampleTime) * pid->dTerm))
               /  (2.0 * pid->tau + pid->sampleTime);
  } else {
    pid->dTerm = 0;
  }

  /*output*/
  pid->output = pid->pTerm + pid->iTerm + pid->dTerm + pid->feedForward;

  /*feedforward*/
  //pid->feedForward = pid->ks * (pid->error - pid->prevError) * (pid->output - pid->prevOutput);

  /*pre saturation output*/
  pid->preSatOutput = pid->output;

  /*output saturation*/
  if (pid->output > pid->maxOutput) pid->output = pid->maxOutput;
  if (pid->output < pid->minOutput) pid->output = pid->minOutput;

  /*saturation error*/
  pid->satError = pid->output - pid->preSatOutput;

  /*update previous variables*/
  pid->prevError = pid->error;
  pid->prevOutput = pid->output;
  pid->lastTime = millis();

  return pid->output;
}

void PidControllerSetKp(PidController_t* pid, float gain) {
  pid->kp = gain * 0.1;
}

void PidControllerSetKi(PidController_t* pid, float gain) {
  pid->ki = gain * 0.1; 
}

void PidControllerSetKd(PidController_t* pid, float gain) {
  pid->kd = gain * 0.1; 
}

void PidControllerSetFFD(PidController_t* pid, float ffd) {
  pid->feedForward = ffd; 
}

float PidGetKp(PidController_t* pid) {
  return pid->kp;
}

float PidGetKi(PidController_t* pid) {
  return pid->ki;
}

float PidGetError(PidController_t* pid) {
  return pid->error;
}

float PidGetPrevError(PidController_t* pid) {
  return pid->prevError;
}

float PidGetpTerm(PidController_t* pid) {
  return pid->pTerm;
}

float PidGetiTerm(PidController_t* pid) {
  return pid->iTerm;
}

float PidGetOutput(PidController_t* pid) {
  return pid->output;
}