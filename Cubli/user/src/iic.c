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
void IIC_Delay(u32 num){
    while (num--);
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
    GPIO_SetBits(SCL_Port,SCL_Pin);
    GPIO_SetBits(SDA_Port,SDA_Pin);
    IIC_Delay(5);
    GPIO_ResetBits(SDA_Port,SDA_Pin);
    IIC_Delay(5);
    GPIO_ResetBits(SCL_Port,SCL_Pin);
}
void IIC_Stop(void){
    SDA_OUT();
    SCL_LOW;
    SDA_LOW;
    IIC_Delay(5);
    
    SCL_HIGH;
    SDA_HIGH;
    IIC_Delay(5);
}
u8 IIC_Wait_Ack(void){
	u8 ucErrTime=0; 
	SDA_IN();
	SDA_HIGH;
    IIC_Delay(5);	  
	while(SDA_READ){
		ucErrTime++;
		if(ucErrTime>50){
			IIC_Stop();
			return 1;
		}
        IIC_Delay(5);
	}  
	SCL_HIGH;
    IIC_Delay(5);
	SCL_LOW;  
	return 0;  
}
void IIC_Ack(void){
    SCL_LOW;
    SDA_OUT();
    SDA_LOW;
    
    IIC_Delay(5);
    SCL_HIGH;
    
    IIC_Delay(5);
    SCL_LOW;
}
void IIC_NAck(void){
    SCL_LOW;
    SDA_OUT();
    SDA_HIGH;
    
    IIC_Delay(5);
    SCL_HIGH;
    
    IIC_Delay(5);
    SCL_LOW;
}
void IIC_Send_Byte(u8 msg){
    u8 i;
    SDA_OUT();
    SCL_LOW;
    for(i=0;i<8;i++){
        if (msg & 0x80){
            SDA_HIGH;
        }else{
            SDA_LOW;
        }
        msg<<=1;
        IIC_Delay(2);
        SCL_HIGH;
        IIC_Delay(5);
        SCL_LOW;
        IIC_Delay(3);
    }
}
u8 IIC_Read_Byte(u8 ack){
    u8 i=0, data=0;
    SDA_IN();
    for(i=0;i<8;i++){
        SCL_LOW;
        IIC_Delay(5);
        SCL_HIGH;
        data <<=1;
        data |= SDA_READ;
        IIC_Delay(5);
    }			 
    if (ack)
        IIC_Ack();
    else
        IIC_NAck();
    return data;
}
void IIC_Send(u8 slaveAddr, u8 regAddr, u8 msg){
    IIC_Start();
    IIC_Send_Byte(slaveAddr);
    IIC_Wait_Ack();
    IIC_Send_Byte(regAddr);
    IIC_Wait_Ack();
    IIC_Send_Byte(msg);
    IIC_NAck();
    IIC_Stop();
}
u8 IIC_Read(u8 slaveAddr, u8 regAddr){
    u8 dataBuffer;
    IIC_Start();
    IIC_Send_Byte(slaveAddr);
    IIC_Wait_Ack();
    IIC_Send_Byte(regAddr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(slaveAddr+1);
    IIC_Wait_Ack();
    dataBuffer = IIC_Read_Byte(0);
    IIC_Stop();
    return dataBuffer; 
}
s16 IIC_Read_Two_Byte(u8 slaveAddr, u8 regAddr){
    u8 data_h = IIC_Read(slaveAddr, regAddr);
    u8 data_l = IIC_Read(slaveAddr, regAddr+1);
    return (data_h<<8)| data_l;
}
void IIC_JY901_Read(u8 slaveAddr, u8 regAddr, u8 length, u8 *msg){
    u8 count = 0;
    IIC_Start();
    IIC_Send_Byte(slaveAddr<<1);
    IIC_Wait_Ack();
    IIC_Send_Byte(regAddr);
    IIC_Wait_Ack();
    IIC_Send_Byte((slaveAddr<<1)+1);
    IIC_Wait_Ack();
    for(count=0;count<length;count++){
        if(count!=length-1)msg[count]=IIC_Read_Byte(1);
        else  msg[count]=IIC_Read_Byte(0);
	}
    IIC_Stop();
}