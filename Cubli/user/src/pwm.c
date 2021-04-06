#include "pwm.h"
/*=========================================================


TIM1 pwm PA8、PA9、PA10、PA11

=========================================================*/

void TM1_PWM_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Alternating functions for pins 原本在PB*/
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);  // chl_1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);  // chl_2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);  // chl_3
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);  // chl_4
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;// | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    //Initialize TIM1
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_BaseStruct.TIM_Period=20000-1;//PWM_frequency = timer_tick_frequency / (TIM_Period + 1) ==> TIM_Period = timer_tick_frequency / PWM_frequency - 1
    TIM_BaseStruct.TIM_Prescaler =180-1;//timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1) 
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);
    
    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
    /*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
    25% duty cycle:      pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
    50% duty cycle:      pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
    75% duty cycle:      pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399*/
    TIM_OCStruct.TIM_Pulse=0;
    TIM_OC1Init(TIM1, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    //
    TIM_OCStruct.TIM_Pulse=0;
    TIM_OC2Init(TIM1, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    //  
    TIM_OCStruct.TIM_Pulse=0;
    TIM_OC3Init(TIM1, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    //  
//    TIM_OCStruct.TIM_Pulse=0;
//    TIM_OC4Init(TIM1, &TIM_OCStruct);
//    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    TIM_Cmd(TIM1,ENABLE);
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

/*=========================================================


TIM2 pwm PA0、PA1、PA2、PA3

=========================================================*/

//void TM2_PWM_Init(void){
//    GPIO_InitTypeDef GPIO_InitStruct;
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//    /* Alternating functions for pins 原本在PB*/
//    //  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);  // chl_1
//    //  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);  // chl_2
//    //  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);  // chl_3
//    //  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);  // chl_4
//    
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);  // chl_1
////    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);  // chl_3
////    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);  // chl_4
//    
//    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;//; | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
//    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStruct);
//    
////    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
////    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
////    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
////    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
////    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
////    GPIO_Init(GPIOB, &GPIO_InitStruct);
//    //Initialize TIM2
//    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
//    TIM_BaseStruct.TIM_Prescaler =90-1;//timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1) 
//    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_BaseStruct.TIM_Period=20000-1;//PWM_frequency = timer_tick_frequency / (TIM_Period + 1) ==> TIM_Period = timer_tick_frequency / PWM_frequency - 1
//    TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
//    TIM_BaseStruct.TIM_RepetitionCounter=0;
//    TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
//    
//    TIM_OCInitTypeDef TIM_OCStruct;
//    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
//    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
//    /*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
//    25% duty cycle:      pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
//    50% duty cycle:      pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
//    75% duty cycle:      pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
//    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399*/
//    TIM_OCStruct.TIM_Pulse=0;
//    TIM_OC1Init(TIM2, &TIM_OCStruct);
//    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
//    //  //  
//    //  TIM_OCStruct.TIM_Pulse=0;
//    //  TIM_OC2Init(TIM2, &TIM_OCStruct);
//    //  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
//    //  
//    TIM_OCStruct.TIM_Pulse=0;
//    TIM_OC3Init(TIM2, &TIM_OCStruct);
//    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
//    //  
//    TIM_OCStruct.TIM_Pulse=0;
//    TIM_OC4Init(TIM2, &TIM_OCStruct);
//    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
//    
//    TIM_Cmd(TIM2,ENABLE);
//}
/*=========================================================


TIM3 pwm PC6、PC7、PC8、PC9

=========================================================*/

//void TM3_PWM_Init(void){
//  GPIO_InitTypeDef GPIO_InitStruct;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//  /* Alternating functions for pins 原本在PC 6 7 8 9*/
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
//  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
//  /*PB 
//  // tim3 ch3、4 at PB GPIO_Pin_0  GPIO_Pin_1;
//  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_TIM3);
//  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_TIM3);
//  */
//  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//GPIO_Pin_8 | GPIO_Pin_9
//  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_Init(GPIOC, &GPIO_InitStruct);
//  
//  //Initialize TIM3
//  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//  TIM_BaseStruct.TIM_Prescaler =0;//timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1) 
//  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_BaseStruct.TIM_Period=8399;//PWM_frequency = timer_tick_frequency / (TIM_Period + 1) ==> TIM_Period = timer_tick_frequency / PWM_frequency - 1
//  TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
//  TIM_BaseStruct.TIM_RepetitionCounter=0;
//  TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
//  TIM_Cmd(TIM3,ENABLE);
//  
//  //Initialize PWM
//  TIM_OCInitTypeDef TIM_OCStruct;
//  /* PWM mode 2 = Clear on compare match */
//  /* PWM mode 1 = Set on compare match */
//  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
//  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
//  /*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
//  25% duty cycle:      pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
//  50% duty cycle:      pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
//  75% duty cycle:      pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
//  100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399*/
//  TIM_OCStruct.TIM_Pulse=2099;
//  TIM_OC1Init(TIM3, &TIM_OCStruct);
//  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
//  
//  TIM_OCStruct.TIM_Pulse=4199;
//  TIM_OC2Init(TIM3, &TIM_OCStruct);
//  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
//  
//  /*TIM_OCStruct.TIM_Pulse=6299;
//  TIM_OC3Init(TIM3, &TIM_OCStruct);
//  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
//  
//  TIM_OCStruct.TIM_Pulse=8399;
//  TIM_OC4Init(TIM3, &TIM_OCStruct);
//  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);*/
//}

/*=========================================================


TIM4 pwm PB6、PB7、PB8、PB9 or PD12、PD13、PD14、PD15

=========================================================*/
//
//void TM4_PWM_Init(void){
//  GPIO_InitTypeDef GPIO_InitStruct;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//  /* Alternating functions for pins 原本在PB*/
//  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
//  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
//  /*
//  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
//  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
//  */
//  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;// | GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_Init(GPIOD, &GPIO_InitStruct);
//  //Initialize TIM4
//  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
//  TIM_BaseStruct.TIM_Prescaler =0;//timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1) 
//  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_BaseStruct.TIM_Period=8399;//PWM_frequency = timer_tick_frequency / (TIM_Period + 1) ==> TIM_Period = timer_tick_frequency / PWM_frequency - 1
//  TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
//  TIM_BaseStruct.TIM_RepetitionCounter=0;
//  TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
//  TIM_Cmd(TIM4,ENABLE);
//  TIM_OCInitTypeDef TIM_OCStruct;
//  /* PWM mode 2 = Clear on compare match */
//  /* PWM mode 1 = Set on compare match */
//  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
//  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
//  /*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
//  25% duty cycle:      pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
//  50% duty cycle:      pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
//  75% duty cycle:      pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
//  100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399*/
//  TIM_OCStruct.TIM_Pulse=2099;
//  TIM_OC1Init(TIM4, &TIM_OCStruct);
//  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
//  
//  TIM_OCStruct.TIM_Pulse=4199;
//  TIM_OC2Init(TIM4, &TIM_OCStruct);
//  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
//  
//  /*TIM_OCStruct.TIM_Pulse=6299;
//  TIM_OC3Init(TIM4, &TIM_OCStruct);
//  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
//  
//  TIM_OCStruct.TIM_Pulse=8399;
//  TIM_OC4Init(TIM4, &TIM_OCStruct);
//  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);*/
//}


/*=========================================================


TIM5 pwm PA0、PA1、PA2、PA3

=========================================================*/

//void TM5_PWM_Init(void){
//  GPIO_InitTypeDef GPIO_InitStruct;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//  /* Alternating functions for pins 原本在PB*/
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
////  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
//  
//  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;// | GPIO_Pin_3;
//  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStruct);
//  //Initialize TIM5
//  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
//  TIM_BaseStruct.TIM_Prescaler =0;//timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1) 
//  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_BaseStruct.TIM_Period=9000-1;//PWM_frequency = timer_tick_frequency / (TIM_Period + 1) ==> TIM_Period = timer_tick_frequency / PWM_frequency - 1
//  TIM_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
//  TIM_BaseStruct.TIM_RepetitionCounter=0;
//  TIM_TimeBaseInit(TIM5, &TIM_BaseStruct);
//  
//  TIM_OCInitTypeDef TIM_OCStruct;
//  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
//  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCStruct.TIM_OCPolarity  = TIM_OCPolarity_High;
//  /*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
//  25% duty cycle:      pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
//  50% duty cycle:      pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
//  75% duty cycle:      pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
//  100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399*/
//  TIM_OCStruct.TIM_Pulse=0;
//  TIM_OC1Init(TIM5, &TIM_OCStruct);
//  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
//  
//  TIM_OCStruct.TIM_Pulse=0;
//  TIM_OC2Init(TIM5, &TIM_OCStruct);
//  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
//  //  
//  TIM_OCStruct.TIM_Pulse=0;
//  TIM_OC3Init(TIM5, &TIM_OCStruct);
//  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
//  
////  TIM_OCStruct.TIM_Pulse=0;
////  TIM_OC4Init(TIM5, &TIM_OCStruct);
////  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
//  TIM_Cmd(TIM5,ENABLE);
//}*/

/*=========================================================


TIM8 pwm PC6、PC7、PC8

=========================================================*/

void TM8_PWM_Init(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;// | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
    //  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
    //时基初始化
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //死区控制用。
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数器方向
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1;   //Timer clock = sysclock /(TIM_Prescaler+1) = 180M
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInitStructure.TIM_Period = 9000 - 1;    //Period = (TIM counter clock / TIM output clock) - 1 = 10K
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
    
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM8,&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM8,&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM8,&TIM_OCInitStructure);
    
    //  TIM_OCInitStructure.TIM_Pulse = 2400;
    //  TIM_OC4Init(TIM8,&TIM_OCInitStructure);
    
    TIM_Cmd(TIM8,ENABLE);
    TIM_CtrlPWMOutputs(TIM8,ENABLE);
}

/*=========================================================

TIM2 EncoderInterface pwm PA2、PA3
TIM3 EncoderInterface pwm PA6、PA7
TIM4 EncoderInterface pwm PB6、PB7

=========================================================*/


int read_Encoder_a(void){
    int Encoder = 0;
    Encoder = (short)TIM2->CNT; //因為設定是65535 所以用short來將逆向的馬達計數轉為負號
    TIM2->CNT = 0;
    return Encoder;
}
int read_Encoder_b(void){
    int Encoder = 0;
    Encoder = (short)TIM3->CNT; //因為設定是65535 所以用short來將逆向的馬達計數轉為負號
    TIM3->CNT = 0;
    return Encoder;
}
int read_Encoder_c(void){
    int Encoder = 0;
    Encoder = (short)TIM4->CNT; //因為設定是65535 所以用short來將逆向的馬達計數轉為負號
    TIM4->CNT = 0;
    return Encoder;
}

//TIM1 EncoderInterface unuse
void TIM1_EncoderInterface_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    // GPIOB Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // Initalize PB5 (TIM1 Ch1) and PB6 (TIM1 Ch2)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;         // Weak Pull-up for safety during startup
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Period =65535;  //reload value
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //無分頻
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上計數
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); // 四分頻stm讀取encoder設置
    TIM_ICStructInit(&TIM_ICInitStructure); 
    TIM_ICInitStructure.TIM_ICFilter = 10;//濾波器值(從第10個訊號後才開始計數)
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清除TIM1的更新旗標
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);//運行更新
    
    //  NVIC_InitTypeDef NVIC_InitStructure;    
    //  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           
    //  
    //  NVIC_InitStructure.NVIC_IRQChannel = TIM1_IRQn;    
    //  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;               
    //  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    //  NVIC_Init(&NVIC_InitStructure);
    TIM_SetCounter(TIM1,0); //TIM1->CNT=0
    TIM_Cmd(TIM1, ENABLE); 
}
//TIM2EncoderInterface
void TIM2_EncoderInterface_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    // GPIOB Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // Initalize PA2 (TIM2 Ch3) and PA3 (TIM2 Ch4)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;         // Weak Pull-up for safety during startup
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Period =65535;  //reload value
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //無分頻
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上計數
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); // 四分頻stm讀取encoder設置
    TIM_ICStructInit(&TIM_ICInitStructure); 
    TIM_ICInitStructure.TIM_ICFilter = 10;//濾波器值(從第10個訊號後才開始計數)
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM3的更新旗標
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//運行更新
    //IM3定時器
    
    //  NVIC_InitTypeDef NVIC_InitStructure;    
    //  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           
    //  
    //  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    
    //  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;               
    //  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    //  NVIC_Init(&NVIC_InitStructure);
    TIM_SetCounter(TIM2,0); //TIM3->CNT=0
    TIM_Cmd(TIM2, ENABLE);  
}
//TIM3EncoderInterface
void TIM3_EncoderInterface_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // GPIOB Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // Initalize PB6 (TIM4 Ch1) and PB7 (TIM4 Ch2)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;         // Weak Pull-up for safety during startup
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Period =65535;  //reload value
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //無分頻
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上計數
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); // 四分頻stm讀取encoder設置
    TIM_ICStructInit(&TIM_ICInitStructure); 
    TIM_ICInitStructure.TIM_ICFilter = 10;//濾波器值(從第10個訊號後才開始計數)
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM3的更新旗標
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//運行更新
    //IM3定時器
    
    //  NVIC_InitTypeDef NVIC_InitStructure;    
    //  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           
    //  
    //  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    
    //  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;               
    //  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    //  NVIC_Init(&NVIC_InitStructure);
    TIM_SetCounter(TIM3,0); //TIM3->CNT=0
    TIM_Cmd(TIM3, ENABLE); 
}
//TIM4 EncoderInterface
void TIM4_EncoderInterface_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // GPIOB Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // Initalize PB14 (TIM4 Ch1) and PB15 (TIM4 Ch2)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;         // Weak Pull-up for safety during startup
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_Period =65535;  //reload value
    TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //無分頻
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上計數
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); // 四分頻stm讀取encoder設置
    TIM_ICStructInit(&TIM_ICInitStructure); 
    TIM_ICInitStructure.TIM_ICFilter = 10;//濾波器值(從第10個訊號後才開始計數)
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM4的更新旗標
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//運行更新
    //IM3定時器
    
    //  NVIC_InitTypeDef NVIC_InitStructure;    
    //  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);           
    //  
    //  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;    
    //  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;               
    //  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    //  NVIC_Init(&NVIC_InitStructure);
    TIM_SetCounter(TIM4,0); //TIM4->CNT=0
    TIM_Cmd(TIM4, ENABLE); 
}