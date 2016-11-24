#ifndef _DATA_H
#define _DATA_H

typedef struct SensorData
{
  float accel[3];  //m2/s
  float gyro[3];   //radio
}xSensorData;

typedef xSensorData SensorData_t;

float CLMAP(float a,float min,float max);
//#define CLMAP(x , min , max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))
typedef struct DataDefinition
{
  SensorData_t* pSensor;
  float q[4];
  float euler[3]; //degrees
}xDATA;

typedef xDATA Data_t;

struct SensorData_t;

extern Data_t Data;

void UpdateSensorData();
void UpdateData();
void DataInit();



#endif