/*
 *  engine.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include <stdint.h>
#include <math.h>
#include "engine.h"
#include "data.h"
#include "control.h"

int debugPrint   = 0;
int debugPerf    = 0;
int debugSense   = 0;
int debugCnt     = 0;
int debugRC      = 0;
int debugOrient  = 0;
int debugAutoPan = 0;

float /*pitch, Gyro_Pitch_angle,*/ pitch_setpoint = 0.0f, pitch_Error_last = 0.0f,  pitch_angle_correction;
float /*roll,  Gyro_Roll_angle,*/  roll_setpoint  = 0.0f,  roll_Error_last = 0.0f,   roll_angle_correction;
float /*yaw,   Gyro_Yaw_angle,*/   yaw_setpoint   = 0.0f,   yaw_Error_last = 0.0f,    yaw_angle_correction;

uint16_t configData[12] = {40,80,20, 150, 100, 30, 30, 150, 60,0 ,0, 64};
//float ADC1Ch13_yaw;


float Output[3];


float AccData[3]  = {0.0f, 0.0f, 0.0f};
float GyroData[3] = {0.0f, 0.0f, 0.0f};

float Step[3]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[3] = {0.0f, 0.0f, 0.0f};

void roll_PID(void)
{
  static float interv=0;
    float Error_current = 0 + Data.euler[0];
    float KP = Error_current * ((float)configData[4])/100.0;
    float KD = ((float)configData[6] ) * (Error_current - roll_Error_last);
    interv+=Error_current;
    float KI = interv*((float)configData[5])/100.0;
    roll_Error_last = Error_current;

    Output[0] = KD + KP+ KI;
    TIM3->CCR2=(sin(Output[0]     )*configData[7])+250; 
    TIM3->CCR4=(sin(Output[0]+2.09)*configData[7])+250;
    TIM3->CCR3=(sin(Output[0]+4.19)*configData[7])+250;
    //SetRollMotor(KP + KD, configData[7]);
}
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
void pitch_PID(void)
{
  static float interv=0;
    float Error_current = -0/57.6 + Data.euler[1];
    
    interv+=Error_current;
    //if(interv>6.28)
      //interv=6.28;
    //if(interv<-6.28)
      
    float KP = Error_current * ((float)configData[0])/100.0;
    float KI = interv*((float)configData[1])/100.0;
    float KD = ((float)configData[2]) * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;

    Output[1] = KD + KP+ KI;
    //SetPitchMotor(Output[1],configData[3]);
    
    
    TIM3->CCR1=(sin(Output[1]     )*configData[3])+250; 
    TIM2->CCR4=(sin(Output[1]+2.09)*configData[3])+250;
    TIM2->CCR3=(sin(Output[1]+4.19)*configData[3])+250;
}

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + Data.euler[2] * 1000.0;
    float KP = Error_current * ((float)configData[2] / 1000.0);
    float KD = ((float)configData[5] / 100.0) * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    Output[2] = KD + KP;
    //SetYawMotor(KP + KD, configData[8]);
}

float constrain(float value, float low, float high)
{
    if (value < low)
        return low;

    if (value > high)
        return high;

    return value;
}

/*
  Limits the Pitch angle
*/


//---------------------YAW autopan----------------------//
//#define ANGLE2SETPOINT -1000
#define DEADBAND 2.0f //in radians with respect to one motor pole (actual angle is (DEADBAND / numberPoles) * R2D)
#define MOTORPOS2SETPNT 0.55f //scaling factor for how fast it should move
#define AUTOPANSMOOTH 40.0f
//#define LPFTIMECONSTANT 20 //change this to adjust sensitivity

//float yawAngleLPF=0;
float centerPoint = 0.0f;
float stepSmooth  = 0.0f;
float step        = 0.0f;

float autoPan(float motorPos, float setpoint)
{

    if (motorPos < centerPoint - DEADBAND)
    {
        centerPoint = (+DEADBAND);
        step = MOTORPOS2SETPNT * motorPos; //dampening
    }
    else if (motorPos > centerPoint + DEADBAND)
    {
        centerPoint = (-DEADBAND);
        step = MOTORPOS2SETPNT * motorPos; //dampening
    }
    else
    {
        step = 0.0f;
        centerPoint = 0.0f;
    }
    stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;
    return (setpoint -= stepSmooth);
}

//--------------------Engine Process-----------------------------//
void engineProcess()
{
    pitch_PID();
    roll_PID();
    //yaw_PID();
}

