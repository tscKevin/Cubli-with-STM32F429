#include "stm32f4xx.h"

#include "timer.h"
#include "pwm.h"

void TM5_Interrupt_Init(void){
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
  TIM_BaseStruct.TIM_Prescaler = 90-1;
  TIM_BaseStruct.TIM_Period=5000-1;
  TIM_BaseStruct.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM5,&TIM_BaseStruct);
  TIM_Cmd(TIM5,ENABLE);
  TIM_ClearFlag(TIM5,TIM_FLAG_Update);
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
}
void NVIC_Set(void){
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//0~2
  
  /*TIM5*/
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;    
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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //enable GPIOA clock   //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;// | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure); //enable GPIOA clock   //
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //enable GPIOA clock   //
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure); //enable GPIOA clock   //
}
int a,b,c;
void main(void){
  systick_setup();
  
  TM2_PWM_Init();//servo
  TIM2->CCR1=1700;//PA0
  TIM2->CCR3=1700;//PB10
  TIM2->CCR4=1700;//PB11
  Delay(150);
  
  TIM3_EncoderInterface_Init();//A
  TIM1_EncoderInterface_Init();//B
  TIM4_EncoderInterface_Init();//C
  Delay(5);
  
  TM8_PWM_Init();
  LED_On_Board();//A:B13,B:D8,C:D9

  IIC_GPIO_Init();
  MPU6050_Init();
  get_mpu_id();
  get_iir_factor(&Mpu.att_acc_factor,0.005f,25);
  
  usart1_init(115200);
  TM5_Interrupt_Init();
  NVIC_Set();
  
  Delay(2000);
  nvic_flag = 1;  /**/
  
  /*
  TIM8->CCR1 = 500;//A PC6
  Delay(1000);
  TIM8->CCR2 = 700;//B PC7
  Delay(1000);
  TIM8->CCR3 = 1000;//C PC8
  */
  
  
  while(1){
//  GPIO_SetBits(GPIOB,GPIO_Pin_13);  //A
////  GPIO_ResetBits(GPIOB,GPIO_Pin_13);
//  
//  GPIO_SetBits(GPIOD,GPIO_Pin_8);  //B
////  GPIO_ResetBits(GPIOD,GPIO_Pin_8);
//  
//  GPIO_SetBits(GPIOD,GPIO_Pin_9);  //C
////  GPIO_ResetBits(GPIOD,GPIO_Pin_9);
//  
//  TIM8->CCR1 = 5500;//A
//  TIM8->CCR2 = 5500;//B
//  TIM8->CCR3 = 5500;//C
//  Delay(2000);
//  
////  GPIO_SetBits(GPIOB,GPIO_Pin_13);
//  GPIO_ResetBits(GPIOB,GPIO_Pin_13);//A
//  
////  GPIO_SetBits(GPIOD,GPIO_Pin_8);
//  GPIO_ResetBits(GPIOD,GPIO_Pin_8);//B
//  
////  GPIO_SetBits(GPIOD,GPIO_Pin_9);
//  GPIO_ResetBits(GPIOD,GPIO_Pin_9);//C
//  
//  TIM8->CCR1 = 10;//A
//  TIM8->CCR2 = 10;//B
//  TIM8->CCR3 = 10;//C
//  Delay(100);
//  TIM8->CCR1 = 0;//A
//  TIM8->CCR2 = 0;//B
//  TIM8->CCR3 = 0;//C
//  Delay(1900);
//  TIM8->CCR1 = 5500;//A
//  TIM8->CCR2 = 5500;//B
//  TIM8->CCR3 = 5500;//C
//  Delay(2000);
//  
//  GPIO_SetBits(GPIOB,GPIO_Pin_13);//A
////  GPIO_ResetBits(GPIOB,GPIO_Pin_13);
//  
//  GPIO_SetBits(GPIOD,GPIO_Pin_8);//B
////  GPIO_ResetBits(GPIOD,GPIO_Pin_8);
//  
//  GPIO_SetBits(GPIOD,GPIO_Pin_9);//C
////  GPIO_ResetBits(GPIOD,GPIO_Pin_9);
//  TIM8->CCR1 = 10;//A
//  TIM8->CCR2 = 10;//B
//  TIM8->CCR3 = 10;//C
//  Delay(100);
//  TIM8->CCR1 = 0;//A
//  TIM8->CCR2 = 0;//B
//  TIM8->CCR3 = 0;//C
//  Delay(1900);
  
//    printf("%d",TIM12->CNT);
    
//  TIM8->CCR1 = 1000;
//  TIM8->CCR2 = 1500;
//  TIM8->CCR3 = 2000;Delay(3000);
  
  
//  TIM8->CCR3=-800;Delay(3000);
  
//    if(att.pit<-27){//jump up
//      Delay(3000);
//      nvic_flag = 0;
//      PWM_x =0;
//      set_pwm(PWM_x);
//      TIM2->CCR3=1250;//1500;
//      Delay(50);
//      nvic_flag = 1;
//      Delay(200);
//      TIM2->CCR3=1700;
//      Delay(200);
//    }else if(att.pit>27){//jump up
//      Delay(3000);
//      nvic_flag = 0;
//      PWM_x =0;
//      set_pwm(PWM_x);
//      TIM2->CCR3=1250;//1500;
//      Delay(50);
//      nvic_flag = 1;
//      Delay(200);
//      TIM2->CCR3=1700;
//      Delay(200);
//    }
  }
}