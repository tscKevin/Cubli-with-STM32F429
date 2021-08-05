#ifndef __I2C_H
#define __I2C_H
 
/****************************************************************
 *                        Header include
*****************************************************************/
#include "stm32f4xx.h"

#include "Delay.h"
 
/****************************************************************
 *                       Macro definition
*****************************************************************/
 
 
/****************************************************************
 *                       Type definition
*****************************************************************/
#define RCC_AHB1Periph_GPIO_Port RCC_AHB1Periph_GPIOB

#define SCL_Port GPIOB
//#define SCL_Pin GPIO_Pin_6
#define SCL_Pin GPIO_Pin_0

#define SDA_Port GPIOB
//#define SDA_Pin GPIO_Pin_7
#define SDA_Pin GPIO_Pin_1

#define SCL_HIGH GPIO_SetBits(SCL_Port,SCL_Pin)
#define SCL_LOW GPIO_ResetBits(SCL_Port,SCL_Pin)

#define SDA_HIGH GPIO_SetBits(SDA_Port,SDA_Pin)
#define SDA_LOW GPIO_ResetBits(SDA_Port,SDA_Pin)

//#define IIC_Delay  Delay(0)

#define SDA_READ ((SDA_Port->IDR &  SDA_Pin)!=0) ? 1 : 0
 
/****************************************************************
 *                     Structure definition
*****************************************************************/
 
 
 
#ifdef __cplusplus
 extern "C" {
#endif  /* __cplusplus */
 
/****************************************************************
 *                     Variable declaration
*****************************************************************/


 
 
/****************************************************************
 *                     Function declaration
*****************************************************************/

void IIC_GPIO_Init(void);
void SDA_OUT(void);
void SDA_IN(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Ack(void);
void IIC_Send_Byte(u8 msg);
u8 IIC_Read_Byte(u8 ack);
void IIC_Send(u8 slaveAddr, u8 regAddr, u8 msg);
u8 IIC_Read(u8 slaveAddr, u8 regAddr);
s16 IIC_Read_Two_Byte(u8 slaveAddr, u8 regAddr);
void IIC_JY901_Read(u8 slaveAddr, u8 regAddr, u8 length, u8 *msg);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
 
#endif	/* __I2C_H */