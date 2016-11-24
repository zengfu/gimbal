#include "stm32f1xx_hal.h"
#include "ANO_DT.h"
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
}

uint8_t b;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  if(huart->Instance==USART3){
    ANO_DT_Data_Receive_Prepare(b);
    //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
    HAL_UART_Receive_IT(huart,&b,1);
  }
}

void HAL_SYSTICK_Callback(void)
{
  ANO_DT_Data_Exchange();
}