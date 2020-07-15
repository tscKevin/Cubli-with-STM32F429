#include "time.h"
void TIM4_IRQHandler(void){
  if(TIM4->SR&0X0001){//確認更新中斷旗標為有效
//    GPIO_ToggleBits(GPIOG,GPIO_Pin_14);
    MPU6050_Get_Display();
  }
  TIM4->SR&=~(1<<0);//清除更新中斷旗標  TIM4->SR = (uint16_t)~TIM_FLAG;
}