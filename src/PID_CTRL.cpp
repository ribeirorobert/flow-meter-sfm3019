#include "PID_CTRL.h"


void PidControllerInit(PidController_Typedef *pid) {
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
}


void PidControllerSetLimits(PidController_Typedef* pid, float minOut, float maxOut) {
  pid->minOutput = minOut;
  pid->maxOutput = maxOut;

  pid->minIntegr = minOut;
  pid->maxIntegr = maxOut;
}


void PidControllerSetGains(PidController_Typedef* pid, float _kp, float _ki, float _kd, float _tau) {
  pid->kp = _kp;
  pid->ki = _ki;
  pid->kd = _kd;
  pid->tau = _tau;
}


void PidControllerUpdate(PidController_Typedef *pid, float reference, float feedback) {
  uint32_t now = micros();
  static uint32_t prevIterationTime;

  float sampleTime = (now - prevIterationTime) * 0.000001f;

  if (sampleTime <= 0.0f || sampleTime > 0.5f) {
    prevIterationTime = now;
    return;
  }

  /*error*/
  pid->error = reference - feedback;

  /*proportional term*/
  if (pid->kp != 0.0f) {
    pid->pTerm = pid->kp * pid->error;
  } else {
    pid->pTerm = 0.0f;
  }

  /*derivative term*/
  if (pid->kd != 0.0f) {
    //dTerm = kd * (error - prevError) / sampleTime;
    pid->dTerm = ((2.0f * pid->kd * (pid->error - pid->prevError))
            + ((2.0f * pid->tau - sampleTime) * pid->dTerm))
            /  (2.0f * pid->tau + sampleTime);
  } else {
    pid->dTerm = 0.0f;
  }

  float u_before = pid->pTerm + pid->iTerm + pid->dTerm;

  /*integration term*/
  if (pid->ki != 0.0f) {
    if (u_before > 0.0f && u_before < 255.0f) {
      pid->iTerm += pid->ki * pid->error * sampleTime;
      pid->iTerm = constrain(pid->iTerm, pid->minIntegr, pid->maxIntegr);
    }
  } else {
    pid->iTerm = 0.0f;
  }

  /*control signal*/
  //pid->output = constrain(pid->pTerm + pid->iTerm + pid->dTerm, pid->minOutput, pid->maxOutput);
  pid->output = pid->pTerm + pid->iTerm + pid->dTerm;

  pid->prevError = pid->error;
  prevIterationTime = now;
}