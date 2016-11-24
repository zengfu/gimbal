#include "control.h"
#include "stm32f1xx_hal.h"
#include "arm_math.h"
#include "data.h"


#define MODE1

Control_t ctrl;
short int sinDataI16[SINARRAYSIZE];
/*
typedef struct
  {
    float32_t A0;          < The derived gain, A0 = Kp + Ki + Kd . 
    float32_t A1;          < The derived gain, A1 = -Kp - 2Kd. 
    float32_t A2;          < The derived gain, A2 = Kd . 
    float32_t state[3];    < The state array of length 3. 
    float32_t Kp;          < The proportional gain.
    float32_t Ki;          < The integral gain. 
    float32_t Kd;          < The derivative gain.
  } arm_pid_instance_f32;

*/
arm_pid_instance_f32 sp_pitch;
arm_pid_instance_f32 sp_roll;
arm_pid_instance_f32 sp_yaw;

arm_pid_instance_f32 pp_pitch;
arm_pid_instance_f32 pp_roll;
arm_pid_instance_f32 pp_yaw;



static void SetPWM(int *pwm, float phi, int power);
static void SetPWMFastTable(int *pwm, float phi, int power);
static void SetPWMData(int *target, int *pwm);

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void MotorInit()
{
  
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);  
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4); //m0 lefe
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); //m0 middle 
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); //m0 right
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//m1 left
  
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); 
  
  
  htim3.Instance->CNT=0;
  htim2.Instance->CNT=0;
  htim4.Instance->CNT=0;
  pp_pitch.Kp=37/1000.0;
  pp_pitch.Kd=6/100.0;
  PidUpate();
  
}
void throttle(uint16_t data)
{
  data=CLMAP(data,1,1200);
  ctrl.power=data;
}
void PidUpate()
{
  arm_pid_init_f32(&sp_pitch,1);
  arm_pid_init_f32(&pp_pitch,1);
  arm_pid_init_f32(&sp_roll,1);
  arm_pid_init_f32(&pp_roll,1);
  arm_pid_init_f32(&sp_yaw,1);
  arm_pid_init_f32(&pp_yaw,1);
}
void ControlUpdate()
{
  float gx,gy,pitch,roll;
  float out1,out2;
  gx=Data.pSensor->gyro[0];
  gy=Data.pSensor->gyro[1];
  roll=Data.euler[0];
  pitch=Data.euler[1]+180;
//out1=arm_pid_f32(&pp_pitch,0-roll*17.45);
  out2=arm_pid_f32(&pp_pitch,30*17.45/57.3-pitch*17.45);
  SetPitchMotor(out2,40);
//TIM3->CCR1=(int)(arm_sin_f32(out2)*200)+500; 
//TIM2->CCR4=(int)(arm_sin_f32(out2+2.09)*200)+500;
  //TIM2->CCR3=(int)(arm_sin_f32(out2+4.19)*200)+500;
  
  
//out1=CLMAP(out1,0,1200);
//out2=CLMAP(out2,0,1200);
//endif
//  MT1=(uint16_t)(ctrl.power+out1+out2);
//  MT2=(uint16_t)(ctrl.power-out1-out2);
//  MT3=(uint16_t)(ctrl.power-out1+out2);
//  MT4=(uint16_t)(ctrl.power+out1+out2);
//  MT1=(uint16_t)(ctrl.power+out1);
//  MT2=(uint16_t)(ctrl.power-out1);
//  MT3=(uint16_t)(ctrl.power-out1);
//  MT4=(uint16_t)(ctrl.power+out1);

}
static int g_Roll[3],g_Pitch[3];
void SetPitchMotor(float phi, int power)
{
    int pwm[3];
    SetPWM(pwm, phi, power);
    SetPWMData(g_Pitch, pwm);
//    TIM3->CCR4=g_Roll[0];//m0 left
//    TIM3->CCR3=g_Roll[1];//m0 middle
//    TIM3->CCR2=g_Roll[2];//m0 right   //roll 
     
    TIM3->CCR1=g_Pitch[0];//m1 left   
    TIM2->CCR4=g_Pitch[2];
    TIM2->CCR3=g_Pitch[1];
  
}
static void SetPWM(int *pwm, float phi, int power)
{
    //SetPWMOrg(pwm, phi, power);
    SetPWMFastTable(pwm, phi, power);
}
static void SetPWMFastTable(int *pwm, float phi, int power)
{

    int phiInt = (int)Round(phi / M_TWOPI * SINARRAYSIZE);
    phiInt = phiInt % SINARRAYSIZE;

    if (phiInt < 0)
    {
        phiInt = SINARRAYSIZE + phiInt;
    }

    int iPower = 5 * power;
    pwm[0] = (sinDataI16[phiInt                          % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + (PWM_PERIODE / 2);
    pwm[1] = (sinDataI16[(phiInt + 1 * SINARRAYSIZE / 3)     % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + (PWM_PERIODE / 2);
    pwm[2] = (sinDataI16[(phiInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + (PWM_PERIODE / 2);
}

static void SetPWMData(int *target, int *pwm)
{

    target[0] = pwm[0];
    target[1] = pwm[1];
    target[2] = pwm[2];

}
float Round(float x)
{
    if (x >= 0)
    {
        return x + 0.5F;
    }
    else
    {
        return x - 0.5F;
    }
}


void InitSinArray(void)
{
    for (int i = 0; i < SINARRAYSIZE; i++)
    {
        float x = i * M_TWOPI / SINARRAYSIZE;
        sinDataI16[i] = (short int)Round(sinf(x) * SINARRAYSCALE);
        //print("i %3d  x %f  sin %d\r\n", i, x, (int)sinDataI16[i]);
    }
}