#include "stm32f4xx.h"
void main(void){
   GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //enable GPIOA clock   //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  while(1){
    //GPIO_SetBits(GPIOG,GPIO_Pin_13);
    //GPIO_SetBits(GPIOG,GPIO_Pin_14);
    
    GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
    //GPIO_ToggleBits(GPIOG, GPIO_Pin_14);

    for(int i=0; i<25000000; i++);
  }

}