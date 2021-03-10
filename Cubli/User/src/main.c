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
    
    /*DMA2_Stream7_IRQn Tx*/
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /*DMA2_Stream1_IRQn Rx*/
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
    NVIC_Init(&NVIC_InitStructure);
    
    /*USART6_IRQn Rx*/
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void GPIO_On_Board(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //enable GPIOA clock   //
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 ;//| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &GPIO_InitStructure); //enable GPIOA clock
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //enable GPIOA clock   //
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;// | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure); //enable GPIOA clock   //
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //enable GPIOA clock   /1/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure); //enable GPIOA clock
}
void EXTI1_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        /* Do your stuff when PD0 is changed */
        if (flag_stop){
            GPIO_SetBits(GPIOG,GPIO_Pin_13);
            flag_stop = 0;
            nvic_flag = 1;
        }else{
            GPIO_ResetBits(GPIOG,GPIO_Pin_13);
            flag_stop = 1;
            nvic_flag = 0;
        }
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
/* Configure pins to be interrupts */
void Configure_PA1(void) {
    /*EXTI0_IRQn	EXTI0_IRQHandler	    Handler for pins connected to line 0
    EXTI1_IRQn	    EXTI1_IRQHandler	    Handler for pins connected to line 1
    EXTI2_IRQn	    EXTI2_IRQHandler	    Handler for pins connected to line 2
    EXTI3_IRQn	    EXTI3_IRQHandler	    Handler for pins connected to line 3
    EXTI4_IRQn	    EXTI4_IRQHandler	    Handler for pins connected to line 4
    EXTI9_5_IRQn	EXTI9_5_IRQHandler	    Handler for pins connected to line 5 to 9
    EXTI15_10_IRQn	EXTI15_10_IRQHandler	Handler for pins connected to line 10 to 15*/
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
    
    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
    
    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}
int a,b,c;

void main(void){
    systick_setup();
    flag_stop = 1;
    nvic_flag = 0;
    GPIO_On_Board();//A:B13,B:D8,C:D9
    usart1_init(115200);
    
    IIC_GPIO_Init();
    MPU6050_Init();//    get_mpu_id();
    get_iir_factor(&Mpu.att_acc_factor,0.005f,25);
    
    TM2_PWM_Init();//servo
    TIM2->CCR1=2000;//PA0
    TIM2->CCR3=1700;//PB10
    TIM2->CCR4=1700;//PB11
    Delay(150);
    
    TIM1_EncoderInterface_Init();//A
    TIM3_EncoderInterface_Init();//B
    TIM4_EncoderInterface_Init();//C
    Delay(5);
    TM8_PWM_Init();
    TM5_Interrupt_Init();
    Configure_PA1();
    NVIC_Set();
    
    Delay(1000);
//    flag_stop = 0;
//    nvic_flag = 1;
    
//    TIM8->CCR1 = 8999*0.4;//A PC6
//    Delay(1000);
//    TIM8->CCR2 = 8999*0.8;//B PC7
//    Delay(1000);
//    TIM8->CCR3 = 8999*0.4;//C PC8
    
    
    while(1){
//          GPIO_SetBits(GPIOG,GPIO_Pin_13);
//          Delay(100);
//          GPIO_ResetBits(GPIOG,GPIO_Pin_13);
//          Delay(100);
        //  printf("%d",usart1_read(a));
//          GPIO_SetBits(GPIOB,GPIO_Pin_13);  //A
//          GPIO_ResetBits(GPIOB,GPIO_Pin_13);
        //  
//          GPIO_SetBits(GPIOD,GPIO_Pin_8);  //B
//          GPIO_ResetBits(GPIOD,GPIO_Pin_8);
        //  
//          GPIO_SetBits(GPIOD,GPIO_Pin_9);  //C
//          GPIO_ResetBits(GPIOD,GPIO_Pin_9);
        //  
        //  TIM8->CCR1 = 5500;//A
        //  TIM8->CCR2 = 5500;//B
        //  TIM8->CCR3 = 5500;//C
        ////  GPIO_SetBits(GPIOB,GPIO_Pin_13);
        //  GPIO_ResetBits(GPIOB,GPIO_Pin_13);//A
        //  
        ////  GPIO_SetBits(GPIOD,GPIO_Pin_8);
        //  GPIO_ResetBits(GPIOD,GPIO_Pin_8);//B
        //  
        ////  GPIO_SetBits(GPIOD,GPIO_Pin_9);
        //  GPIO_ResetBits(GPIOD,GPIO_Pin_9);//C
        //  
//          TIM8->CCR1 = 10;//A
//          TIM8->CCR2 = 10;//B
//          TIM8->CCR3 = 10;//C
//          Delay(100);
//          TIM8->CCR1 = 0;//A
//          TIM8->CCR2 = 0;//B
//          TIM8->CCR3 = 0;//C
//          Delay(1900);
//          TIM8->CCR1 = 5500;//A
//          TIM8->CCR2 = 5500;//B
//          TIM8->CCR3 = 5500;//C
//          Delay(2000);
        //  
        //  GPIO_SetBits(GPIOB,GPIO_Pin_13);//A
        ////  GPIO_ResetBits(GPIOB,GPIO_Pin_13);
        //  
        //  GPIO_SetBits(GPIOD,GPIO_Pin_8);//B
        ////  GPIO_ResetBits(GPIOD,GPIO_Pin_8);
        //  
//          GPIO_SetBits(GPIOD,GPIO_Pin_9);//C
        ////  GPIO_ResetBits(GPIOD,GPIO_Pin_9);
        //  TIM8->CCR1 = 10;//A
        //  TIM8->CCR2 = 10;//B
        //  TIM8->CCR3 = 10;//C
        //  Delay(100);
        //  TIM8->CCR1 = 0;//A
        //  TIM8->CCR2 = 0;//B
        //  TIM8->CCR3 = 0;//C
//          Delay(1900);
        
        //    printf("%d",TIM12->CNT);
        
        //  TIM8->CCR1 = 1000;
        //  TIM8->CCR2 = 1500;
        //  TIM8->CCR3 = 2000;Delay(3000);
        
        
        //  TIM8->CCR3=-800;Delay(3000);
        //  
        //    if(att.rol<-80){//jump up
        ////      Delay(3000);
        //////      nvic_flag = 0;
        //////      PWM_a =0;
        //////      set_pwm_a(PWM_a);
        ////      TIM2->CCR1=1400;//1500;
        //////      Delay(50);
        //////      nvic_flag = 1;
        ////      Delay(200);
        ////      TIM2->CCR1=2000;
        ////      Delay(200);
        //    }else if(att.rol>-27){//jump up
        //      Delay(3000);
        ////      nvic_flag = 0;
        ////      PWM_a =0;
        ////      set_pwm_a(PWM_a);
        //      TIM2->CCR1=1400;//1500;
        ////      Delay(50);
        ////      nvic_flag = 1;
        //      Delay(200);
        //      TIM2->CCR1=2000;
        //      Delay(150);
        //    }
    }
}