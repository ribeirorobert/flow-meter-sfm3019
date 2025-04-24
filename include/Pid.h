#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>


struct PidController_s {
  float reference;
  float feedback;
  float prevFeedback;

  float feedForward;

  float error;
  float prevError;
  float satError;
  float maxError;
  float minError;

  float pTerm;
  float iTerm;
  float dTerm;

  float kp;
  float ki;
  float kd;
  float ks;
  float ku;

  float tau;

  float maxOutput;
  float minOutput;
  float output;
  float preSatOutput;
  float prevOutput;

  float maxIntegr;
  float minIntegr;

  float sampleTime;
  unsigned long lastTime;
};

typedef PidController_s   PidController_t;


/**
 * @brief 
 * 
 * @param 
 * @return nothing
*/
void PidControllerInit(PidController_t*);


/**
 * @brief 
 * 
 * @param 
 * @return nothing
*/
void PidControllerSetGains(PidController_t* pid, float Kp, float Ki, float Kd, float Ks, float Ku, float Tau);


/**
 * @brief 
 * 
 * @param 
 * @return nothing
*/
void PidControllerSetLimits(PidController_t* pid, float maxError, float minError, float maxInt, float minInt, float maxOut, float minOut);


/**
 * @brief 
 * 
 * @param 
 * @return nothing
*/
float PidControllerUpdate(PidController_t* pid, float reference, float feedback);

void PidControllerSetKp(PidController_t* pid, float gain);

void PidControllerSetKi(PidController_t* pid, float gain);

void PidControllerSetKd(PidController_t* pid, float gain);

void PidControllerSetFFD(PidController_t* pid, float ffd);

float PidGetKp(PidController_t* pid);

float PidGetKi(PidController_t* pid);

float PidGetError(PidController_t* pid);

float PidGetPrevError(PidController_t* pid);

float PidGetOutput(PidController_t* pid);

float PidGetpTerm(PidController_t* pid);

float PidGetiTerm(PidController_t* pid);

#endif