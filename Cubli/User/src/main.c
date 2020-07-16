#include "stm32f4xx.h"

#include "timer.h"
#include "pwm.h"
void TM4_Interrupt_Init(void){
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  TIM_BaseStruct.TIM_Prescaler = 90-1;
  TIM_BaseStruct.TIM_Period=5000-1;
  TIM_BaseStruct.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM4,&TIM_BaseStruct);
  TIM_Cmd(TIM4,ENABLE);
  TIM_ClearFlag(TIM4,TIM_FLAG_Update);
  TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
}
void NVIC_Set(void){
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//0~2
  
  /*TIM4*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;    
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
  NVIC_Init(&NVIC_InitStructure);
  
  /*DMA1_Stream6_IRQn*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  NVIC_Init(&NVIC_InitStructure);
}
void LED_On_Board(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //enable GPIOA clock   //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}
void main(void){
  systick_setup();
  TM2_PWM_Init();
  TIM2->CCR3=1400;
  Delay(150);
  TIM2->CCR3=1700;Delay(150);
  TIM3_EncoderInterface_Init();
  Delay(5);
  TM5_PWM_Init();
  LED_On_Board();
  IIC_GPIO_Init();
  
  MPU6050_Init();
  get_mpu_id();
  
  get_iir_factor(&Mpu.att_acc_factor,0.005f,25);
  TM4_Interrupt_Init();
  NVIC_Set();
  Delay(2000);
  nvic_flag = 1;  //while(1);
  while(1){
    if(att.pit<-27){//jump up
      Delay(3000);
      nvic_flag = 0;
      PWM_X =0;
      set_pwm(PWM_X);
      TIM2->CCR3=1250;//1500;
      Delay(50);
      nvic_flag = 1;
      Delay(200);
      TIM2->CCR3=1700;
      Delay(200);
//      
    }else if(att.pit>27){//jump up
      Delay(3000);
      nvic_flag = 0;
      PWM_X =0;
      set_pwm(PWM_X);
      TIM2->CCR3=1250;//1500;
      Delay(50);
      nvic_flag = 1;
      Delay(200);
      TIM2->CCR3=1700;
      Delay(200);
    }
  }
}