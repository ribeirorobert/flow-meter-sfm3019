#ifndef _PID_CTRL_H_
#define _PID_CTRL_H_

#include <Arduino.h>


#define TAU_D   0.001f

typedef struct {
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
} PidController_Typedef;


/**
 * @brief Initialize PID controller
 * 
 * @param 
 * @return nothing
*/
void PidControllerInit(PidController_Typedef *pid);


/**
 * @brief Set gains
 * 
 * @param 
 * @return nothing
*/
void PidControllerSetLimits(PidController_Typedef* pid, float minOut, float maxOut);


/**
 * @brief 
 * 
 * @param 
 * @return nothing
*/
void PidControllerSetGains(PidController_Typedef* pid, float _kp, float _ki, float _kd, float _tau);


/**
 * @brief Run the controller
 * 
 * @param 
 * @return nothing
*/
void PidControllerUpdate(PidController_Typedef *pid, float reference, float feedback);

#endif