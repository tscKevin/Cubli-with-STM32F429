#include "stm32f4xx.h"
#include "timer.h"
#include "pwm.h"

// main timer interrupt, 5ms for ones time
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIOB
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //GPIOD
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //GPIOG
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Do your stuff when PD1 is changed */
        if (!nvic_flag){
            GPIO_SetBits(GPIOG,GPIO_Pin_13);
            nvic_flag = 1;
            jump_state = 1;
        }else{
            GPIO_ResetBits(GPIOG,GPIO_Pin_13);
            nvic_flag = 0;
        }
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/* Configure pins to be interrupts */
void Configure_PA0(void) {
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
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    
    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
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
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

void Brake_system(int enable, int servo){
    /*servo a PA8, b PA9, c PA10*/
    if (enable==1){
        if (servo==1){
            TIM1->CCR1=2600;
        }else if (servo==23){
            TIM1->CCR2=2600;
            TIM1->CCR3=1550;
        }else if (servo==123){
            TIM1->CCR1=2600;
            TIM1->CCR2=2600;
            TIM1->CCR3=1550;
        }
    }else{
        if (servo==1){
            TIM1->CCR1=1550;
        }else if (servo==23){
            TIM1->CCR2=1450;
            TIM1->CCR3=1880;
        }else if (servo==123){
            TIM1->CCR1=1550;
            TIM1->CCR2=1450;
            TIM1->CCR3=1880;
        }
    }
}
void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}
short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}
unsigned char chrTemp[6];
float a=0;
void main(void){
    systick_setup();
    nvic_flag = 0;
    
    GPIO_On_Board();//A:B13,B:D8,C:D9
    usart1_init(115200);
    IIC_GPIO_Init();
    
    MPU6050_Init();//get_mpu_id();
    get_iir_factor(&Mpu.att_acc_factor,0.005f,25);
    
    TM1_PWM_Init();//servo
    Brake_system(0,123);
    TM8_PWM_Init();
    
    TIM2_EncoderInterface_Init();//A
    TIM3_EncoderInterface_Init();//B
    TIM4_EncoderInterface_Init();//C
    Delay(5);
    TM5_Interrupt_Init();
    Configure_PA0();
    NVIC_Set();
    
    while(1){     
        /* LED AND MOTOR FORWORD AND BACKWORD TEST
        printf("%d",usart1_read(a));
        GPIO_SetBits(GPIOG,GPIO_Pin_13); // LED
        GPIO_SetBits(GPIOB,GPIO_Pin_13); //A
        GPIO_SetBits(GPIOD,GPIO_Pin_8);  //B
        GPIO_SetBits(GPIOD,GPIO_Pin_9);  //C
        Delay(100);        
        GPIO_ResetBits(GPIOG,GPIO_Pin_13);
        GPIO_ResetBits(GPIOB,GPIO_Pin_13);
        GPIO_ResetBits(GPIOD,GPIO_Pin_8);
        GPIO_ResetBits(GPIOD,GPIO_Pin_9);*/
        
        /* MOTOR PWM TEST
        TIM8->CCR1 = 8999*0.03;//A PC6
        Delay(1000);
        TIM8->CCR2 = 8999*0.8;//B PC7
        Delay(1000);
        TIM8->CCR3 = 8999*0.4;//C PC8
        Delay(500);
        TIM8->CCR1 = 10;//A
        TIM8->CCR2 = 10;//B
        TIM8->CCR3 = 10;//C
        Delay(500);
        TIM8->CCR1 = 0;//A
        TIM8->CCR2 = 0;//B
        TIM8->CCR3 = 0;//C
        Delay(500);
        TIM8->CCR1 = 5500;//A
        TIM8->CCR2 = 5500;//B
        TIM8->CCR3 = 5500;//C
        Delay(500);
        TIM8->CCR1 = 0;//A
        TIM8->CCR2 = 0;//B
        TIM8->CCR3 = 0;//C
        Delay(500);*/

        /* MMOTOR ADDED TEST
        if (jump_pwm<8000){
        jump_pwm+=25;
        Delay(500);
        }else if (jump_pwm>8000){
        jump_pwm=0;
        }*/

        /* servo motor test
        Brake_system(1,1);
        Delay(250);
        Brake_system(0,1);
        Delay(2250);
        Brake_system(1,23);
        Delay(250);
        Brake_system(0,23);
        Delay(2250);        
        */        
        
        if (nvic_flag==1){
            if((att.rol<-15 || att.rol>15) && (jump_state==1)){//2D jump up
                Delay(6000);
                if(!(att.rol<-15)) return;// || att.rol>15)) return;
                nvic_flag = 0;
                PWM_a = 0;
                set_pwm(1,0,0,0);
                Brake_system(1,1);
                int wait = 150;
                while((att.rol<-15) && --wait>0) Delay(1);//|| att.rol>9
                nvic_flag = 1;
                Brake_system(0,1);
                Delay(2000);
                Delay(2000);
                if (att.rol>=-9.0 && att.rol <=9.0){
                    while(int_abs(encoder_a)>=400) Delay(1);
                    jump_pwm = 0;
                    jump_pwm_max = 4000;
                    jump_state=2;
                }else{
                    jump_state=1;
                    set_pwm(1,0,0,0);
                }
            }
            if(att.rol>=-9.0 && att.rol <=9.0 && att.pit > 8 && jump_state==2){//3D jump up
                if (jump_pwm>=jump_pwm_max){
                    Delay(6000);
                    Brake_system(1,23);
                    Delay(20);
                    jump_state=0;
                    int wait = 150;
                    while((att.pit>15) && --wait>0) Delay(1);//|| att.rol>9
                    Brake_system(0,23);
                    jump_pwm=0;
                    jump_pwm_max=0;
                    Delay(2000);
                }
            }
        }
    }
}