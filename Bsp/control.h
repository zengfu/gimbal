#ifndef _CONTROL_H
#define _CONTROL_H


#include "stm32f1xx_hal.h"
#include "arm_math.h"


extern arm_pid_instance_f32 sp;
extern arm_pid_instance_f32 pp;

typedef struct ControlDefine
{
  uint32_t power;
  uint32_t pitch;
  uint32_t roll;
  uint32_t yaw;
}xControl;

typedef xControl Control_t;

#define SINARRAYSIZE 1024
#define SINARRAYSCALE 32767
#define M_TWOPI 6.28
#define PWM_PERIODE 1000


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern Control_t ctrl;

void MotorInit();
void throttle(uint16_t data);
void PidUpate();
void ControlUpdate();
float Round(float x);
void InitSinArray(void);
void SetPitchMotor(float phi, int power);

extern arm_pid_instance_f32 sp_pitch;
extern arm_pid_instance_f32 sp_roll;
extern arm_pid_instance_f32 sp_yaw;

extern arm_pid_instance_f32 pp_pitch;
extern arm_pid_instance_f32 pp_roll;
extern arm_pid_instance_f32 pp_yaw;


#endif