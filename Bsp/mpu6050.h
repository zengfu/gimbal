#ifndef _MPU_6050_H
#define _MPU_6050_H


#include "stm32f1xx_hal.h"

void MPU6050_Get6AxisRawData(int16_t *accel,int16_t* gyro);
uint8_t MPU_Init();

#endif