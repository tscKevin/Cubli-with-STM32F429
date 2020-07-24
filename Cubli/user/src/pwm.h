#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

void TM2_PWM_Init(void);
void TM3_PWM_Init(void);
void TM8_PWM_Init(void);
void TIM3_EncoderInterface_Init(void);
void TIM4_EncoderInterface_Init(void);
void TIM1_EncoderInterface_Init(void);
int read_Encoder_x(void);
int read_Encoder_y(void);
int read_Encoder_z(void);