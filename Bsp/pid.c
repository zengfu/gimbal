#include "pid.h"

#define DEFAULT_PID_INTEGRATION_LIMIT  500



void pidInit(PID_TypeDef* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt        = dt;
}


float pidUpdate(PID_TypeDef *pid,float measured,u8 careTime)
{
  float output;
  pid->error=pid->desired-measured;
  
  if(careTime)
    pid->integ+=pid->error*pid->dt;
  else
    pid->integ+=pid->error;
  
  if(pid->integ>pid->iLimit)
  {
    pid->integ=pid->iLimit;
  }
  else if(pid->integ<pid->iLimitLow)
  {
    pid->integ=pid->iLimitLow;
  }
  else
  {
  }
  if(careTime)
  {
    pid->deriv=(pid->error-pid->prevError)/pid->dt;
  }
  else
  {
    pid->deriv=pid->error-pid->prevError;
  }
  pid->outI=pid->integ*pid->ki;
  pid->outD=pid->deriv*pid->kd;
  pid->outP=pid->error*pid->kp;
  
  output=pid->outD+pid->outI+pid->outP;
  
  pid->prevError=pid->error;
  return output;
}
void pidSetIntegralLimit(PID_TypeDef* pid, const float limit)
{
    pid->iLimit = limit;
}


void pidSetIntegralLimitLow(PID_TypeDef* pid, const float limitLow)
{
    pid->iLimitLow = limitLow;
}

void pidReset(PID_TypeDef* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetDesired(PID_TypeDef* pid, const float desired)
{
  pid->desired = desired;
}

void pidSetKp(PID_TypeDef* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PID_TypeDef* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PID_TypeDef* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PID_TypeDef* pid, const float dt) 
{
    pid->dt = dt;
}