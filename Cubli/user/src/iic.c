#include "iic.h"

void IIC_GPIO_Init(void){
  GPIO_InitTypeDef iic_gpio;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_Port,ENABLE);
  iic_gpio.GPIO_Pin = SCL_Pin | SDA_Pin;
  iic_gpio.GPIO_Mode = GPIO_Mode_OUT;
  iic_gpio.GPIO_OType = GPIO_OType_PP;
  iic_gpio.GPIO_PuPd = GPIO_PuPd_UP;
  iic_gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&iic_gpio);
  SCL_HIGH;
  SDA_HIGH;
}

static void IIC_Delay(void){
//  Delay(1);
}

void SDA_OUT(void){
  GPIO_InitTypeDef iic_gpio;
  iic_gpio.GPIO_Pin = SDA_Pin;
  iic_gpio.GPIO_PuPd = GPIO_PuPd_UP;
  iic_gpio.GPIO_Mode = GPIO_Mode_OUT;
  iic_gpio.GPIO_OType = GPIO_OType_PP;
  iic_gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&iic_gpio);
}

void SDA_IN(void){
  GPIO_InitTypeDef iic_gpio;
  iic_gpio.GPIO_Pin = SDA_Pin;
  iic_gpio.GPIO_PuPd = GPIO_PuPd_UP;
  iic_gpio.GPIO_Mode = GPIO_Mode_IN;
  iic_gpio.GPIO_OType = GPIO_OType_PP;
  iic_gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&iic_gpio);
}

void IIC_Start(void){
  SDA_OUT();
  SCL_HIGH;
  SDA_HIGH;IIC_Delay;
  
  SDA_LOW;IIC_Delay;
}

void IIC_Stop(void){
  SDA_OUT();
  SCL_LOW;IIC_Delay;
  SDA_LOW;IIC_Delay;
  
  SCL_HIGH;IIC_Delay;
  SDA_HIGH;IIC_Delay;
}

u8 IIC_Ack(void){
  SDA_OUT();
  SCL_LOW;IIC_Delay;
  SDA_HIGH;
  
  SDA_IN();IIC_Delay;
  SCL_HIGH;IIC_Delay;
  
  if(SDA_READ){
    SCL_LOW;
    return 0;
  }
  
  SCL_LOW;IIC_Delay;
  return 1;
}

void IIC_Send_Byte(u8 msg){
  u8 i = 8;
  SDA_OUT();
  while(i--){
    SCL_LOW;IIC_Delay;
    
    if (msg & 0x80){
      SDA_HIGH;
    }else{
      SDA_LOW;
    }
    msg<<=1;IIC_Delay;
    SCL_HIGH;IIC_Delay;
  }
  
  SCL_LOW;
  if (IIC_Ack()==0){
    return;
  }
}

u8 IIC_Read_Byte(void){
  u8 i=0, data=0;
  SDA_IN();
  for(i=0;i<8;i++){
    data <<=1;
    SCL_HIGH;IIC_Delay;
    data |= SDA_READ;
    SCL_LOW;IIC_Delay;
  }
  return data;
}

void IIC_Send(u8 slaveAddr, u8 regAddr, u8 msg){
  IIC_Start();
  IIC_Send_Byte(slaveAddr);
  IIC_Send_Byte(regAddr);
  IIC_Send_Byte(msg);
  IIC_Stop();  
}

u8 IIC_Read(u8 slaveAddr, u8 regAddr){
  u8 dataBuffer;
  IIC_Start();
  IIC_Send_Byte(slaveAddr);
  IIC_Send_Byte(regAddr);
  IIC_Start();
  IIC_Send_Byte(slaveAddr+1);
  dataBuffer = IIC_Read_Byte();
  IIC_Ack();
  IIC_Stop();
  return dataBuffer; 
}

s16 IIC_Read_Two_Byte(u8 slaveAddr, u8 regAddr){
  u8 data_h = IIC_Read(slaveAddr, regAddr);
  u8 data_l = IIC_Read(slaveAddr, regAddr+1);
  return (data_h<<8)| data_l;
}