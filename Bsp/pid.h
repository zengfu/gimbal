#ifndef PID_H
#define PID_H

#include "comm.h"

typedef struct
{
  float desired;     //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit
  float iLimitLow;    //< integral limit
  float dt;           //< delta-time dt
#ifdef PID_EX
  float power;
#endif
}PID_TypeDef;

void pidSetDt(PID_TypeDef* pid, const float dt);
void pidSetKd(PID_TypeDef* pid, const float kd);
void pidSetKi(PID_TypeDef* pid, const float ki);
void pidSetKp(PID_TypeDef* pid, const float kp);
void pidSetDesired(PID_TypeDef* pid, const float desired);
void pidReset(PID_TypeDef* pid);
void pidSetIntegralLimitLow(PID_TypeDef* pid, const float limitLow);
void pidSetIntegralLimit(PID_TypeDef* pid, const float limit);
float pidUpdate(PID_TypeDef *pid,float measured,u8 careTime);
void pidInit(PID_TypeDef* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt);






#endif