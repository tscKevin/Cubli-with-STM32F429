#ifndef _usart_h_
#define _usart_h_

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mpu6050.h"
#include "imu.h"
#include "timer.h"
#include "pwm.h"
#include "Delay.h"

void usart1_init(u32 bound);
void usart1_send(void* buf, int len);
//void dma_tx_config(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length);
void ANO_DT_Send_Status(void);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(float votage, float current);
void ANO_DT_Send_PWM_Motor(u16 M1,u16 M2,u16 M3,u16 M4,u16 M5,u16 M6,u16 M7,u16 M8);
void Anotc_SendData(void);

#endif



