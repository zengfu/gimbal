#include "mpu6050.h"
#include "stm32f1xx_hal.h"



#define MPU_ADDR 0xd0  //0xd0
#define DEV  hi2c1

extern I2C_HandleTypeDef hi2c1;

uint8_t MPU_Init()
{
  uint8_t data;
  
  HAL_I2C_Mem_Read(&DEV,MPU_ADDR,0x75,I2C_MEMADD_SIZE_8BIT,&data,1,100);
  if(data!=0x68) return 1;
  data=0;//sample data rate =1K or 8k(dlfp is enabled)
  HAL_I2C_Mem_Write(&DEV,MPU_ADDR,0x19,I2C_MEMADD_SIZE_8BIT,&data,1,100);
  data=0x03;//dlfp accel 44hz, gyro 42hz
  HAL_I2C_Mem_Write(&DEV,MPU_ADDR,0x1A,I2C_MEMADD_SIZE_8BIT,&data,1,100);
  data=0x03<<3;//gyro +-2000
  HAL_I2C_Mem_Write(&DEV,MPU_ADDR,0x1B,I2C_MEMADD_SIZE_8BIT,&data,1,100);
  data=0;//accel +-2g
  HAL_I2C_Mem_Write(&DEV,MPU_ADDR,0x1C,I2C_MEMADD_SIZE_8BIT,&data,1,100);
  data=0x01;//choose the x gyo pll as the clock
  HAL_I2C_Mem_Write(&DEV,MPU_ADDR,0x6b,I2C_MEMADD_SIZE_8BIT,&data,1,100);
  
  return 0;
}
void MPU6050_Get6AxisRawData(int16_t *accel,int16_t* gyro)
{
  uint8_t data[14];
  HAL_I2C_Mem_Read(&DEV,MPU_ADDR,0x3b,I2C_MEMADD_SIZE_8BIT,data,14,100);
  accel[0] = (data[0] << 8) | data[1];
  accel[1] = (data[2] << 8) | data[3];
  accel[2] = (data[4] << 8) | data[5];

  gyro[0] = (data[8] << 8) | data[9];
  gyro[1] = (data[10] << 8) | data[11];
  gyro[2] = (data[12] << 8) | data[13];
  
}
