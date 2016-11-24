#include "stm32f1xx_hal.h"
#include "data.h"
#include "mpu6050.h"
#include "arm_math.h"
#include "imu.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


static void Quat2Euler();
static void Euler2Quat();

static SensorData_t SensorData;

Data_t Data;

void DataInit()
{
  float roll,pitch;
  //TODO: the part need low filter to reduce tolerance
  UpdateSensorData();
  Data.pSensor=&SensorData;
  roll=atan2(Data.pSensor->accel[1],Data.pSensor->accel[2]);
  pitch=-atan2(Data.pSensor->accel[0],Data.pSensor->accel[2]);
  //
  //pitch=(asin(CLMAP(Data.pSensor->accel[1] / 9.8,-1.0,1.0)));
  Data.euler[0]=roll;
  Data.euler[1]=pitch;
  Data.euler[2]=0.0;
  Euler2Quat();
}
void UpdateData()
{
  UpdateSensorData();//get the new data;
  IMUupdate();//caculate the pitch and roll
  Quat2Euler();//
}

void Quat2Euler()
{
  float q0,q1,q2,q3;
  float pitch,roll,yaw;
  q0=Data.q[0];
  q1=Data.q[1];
  q2=Data.q[2];
  q3=Data.q[3];
 
   
  //roll=asin(CLMAP(2 * (q2 * q3 + q0 * q1) , -1.0f , 1.0f));
  //pitch=-atan2(2 * (q1 * q3 - q0* q2) , 1- 2 * (q2 * q2+ q1 * q1));
  //yaw=atan2(my*arm_cos_f32(roll)+mx*arm_sin_f32(roll)*arm_sin_f32(pitch)-mz*arm_sin_f32(roll)*arm_cos_f32(pitch),mx*arm_cos_f32(pitch)-mz*arm_sin_f32(pitch))*57.3;
  //yaw = -(0.9 * (-yaw + gz*0.002*57.3) + 5.73* atan2(mx*cos(roll) + my*sin(roll)*sin(pitch) + mz*sin(roll)*cos(pitch), my*cos(pitch) - mz*sin(pitch)));
  //yaw=atan2(2 * (q0 * q2 + q3 * q1) , 1 - 2 * (q1 * q1 + q2 * q2))*57.3; 
  //yaw=atan2(mx,my)*57.3;
  
  //Data.euler[0] = roll; 
  //Data.euler[1]  = pitch;
  //Data.euler[2] =0.9*(yaw1+gz*57.3*0.002)+0.1*yaw;
  //Data.euler[2] = atan2(2 * (q0 * q2 + q3 * q1) , 1 - 2 * (q1 * q1 + q2 * q2)); 
  //Data.euler[2]=atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
  //Data.euler[2] =0.9*(yaw-gz*57.3*0.002)+0.1*yaw;
  //Data.euler[2]=yaw;
  roll=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  pitch=asin(CLMAP(2*(q0*q2-q1*q3),-1.0f,1.0f));
  yaw=atan2(2*(q0*q3+q1*q2),1-2*(q3*q3+q2*q2));
  
  Data.euler[0]=roll; 
  Data.euler[1]=pitch;
  Data.euler[2]=yaw; 
}  

void Euler2Quat()
{
  
  float recipNorm;
  float fCosHRoll = arm_cos_f32(Data.euler[0] * .5f);
  float fSinHRoll = arm_sin_f32(Data.euler[0] * .5f);
  float fCosHPitch = arm_cos_f32(Data.euler[1] * .5f);
  float fSinHPitch = arm_sin_f32(Data.euler[1] * .5f);
  float fCosHYaw = arm_cos_f32(Data.euler[2] * .5f);
  float fSinHYaw = arm_sin_f32(Data.euler[2]* .5f);
  float q0,q1,q2,q3;

  /// Cartesian coordinate System
  q0 = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
  q1 = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
  q2 = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
  q3 = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;

//  q0 = fCosHRoll * fCosHPitch * fCosHYaw - fSinHRoll * fSinHPitch * fSinHYaw;
//  q1 = fCosHRoll * fSinHPitch * fCosHYaw - fSinHRoll * fCosHPitch * fSinHYaw;
//  q2 = fSinHRoll * fCosHPitch * fCosHYaw + fCosHRoll * fSinHPitch * fSinHYaw;
//  q3 = fCosHRoll * fCosHPitch * fSinHYaw + fSinHRoll * fSinHPitch * fCosHYaw;
        
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  Data.q[0]=q0;
  Data.q[1]=q1;
  Data.q[2]=q2;
  Data.q[3]=q3;      
}

void UpdateSensorData()
{
  short accel[3],gyro[3];
  MPU6050_Get6AxisRawData(accel, gyro);
  SensorData.accel[0]=accel[0]/32768.0*2.0*9.8;//m2/s
  SensorData.accel[1]=accel[1]/32768.0*2.0*9.8;
  SensorData.accel[2]=accel[2]/32768.0*2.0*9.8;
  SensorData.gyro[0]=gyro[0]/32768.0*2000/57.32;//radio
  SensorData.gyro[1]=gyro[1]/32768.0*2000/57.32;
  SensorData.gyro[2]=gyro[2]/32768.0*2000/57.32;
}
float CLMAP(float a,float min,float max)
{
  if(a<min)
    a=min;
  else if(a>max)
    a=max;
  else
    a=a;
  return a;
}





