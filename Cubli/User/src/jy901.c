#include "jy901.h"
void IIC_GPIO_Init(void){
    GPIO_InitTypeDef iic_gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    iic_gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    iic_gpio.GPIO_Mode = GPIO_Mode_OUT;
    iic_gpio.GPIO_OType = GPIO_OType_PP;
    iic_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&iic_gpio);
	SDA_OUT();     //sda??�X
	SDA_HIGH;	  	  
	SCL_HIGH;
}
//void Delay(u32 count)//�Τ_?��400KHzIIC�H?�һݭn����?
//{
//	while (count--);
//}
//static void IIC_Delay(void){
//    //  Delay(1);
//}
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
/**************************??��?********************************************
*��?�쫬:		void IIC_Start(void)
*�\�@�@��:		?��IIC�_�l�H?
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda??�X
	SDA_HIGH;	  	  
	SCL_HIGH;
	
	Delay(5);
 	SDA_LOW;//START:when CLK is high,DATA change form high to low 
	
	Delay(5);
	SCL_LOW;//?��I2C??�A��??�e�α���?�u 
}

/**************************??��?********************************************
*��?�쫬:		void IIC_Stop(void)
*�\�@�@��:	    //?��IIC����H?
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda??�X
	SCL_LOW;
	SDA_LOW;//STOP:when CLK is high DATA change form low to high
 	
    Delay(5);
	SCL_HIGH; 
	SDA_HIGH;//?�eI2C???���H?
	
    Delay(5);							   	
}

/**************************??��?********************************************
*��?�쫬:		u8 IIC_Wait_Ack(void)
*�\�@�@��:	    ����?���H?��? 
//��^�ȡG1�A����?����?
//        0�A����?�����\
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	SDA_IN();      //SDA?�m??�J  
	SDA_HIGH;
    Delay(5);	  
	while(SDA_READ)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
		Delay(5);
	}  
	SCL_HIGH;
	Delay(5); 
	SCL_LOW;//???�X0  
	return 0;  
} 

/**************************??��?********************************************
*��?�쫬:		void IIC_Ack(void)
*�\�@�@��:	    ?��ACK?��
*******************************************************************************/
void IIC_Ack(void)
{
	SCL_LOW;
	SDA_OUT();
	SDA_LOW;
    Delay(5);
	SCL_HIGH;
    Delay(5);
	SCL_LOW;
}

/**************************??��?********************************************
*��?�쫬:		void IIC_NAck(void)
*�\�@�@��:	    ?��NACK?��
*******************************************************************************/	    
void IIC_NAck(void)
{
	SCL_LOW;
	SDA_OUT();
	SDA_HIGH;
	
    Delay(5);
	SCL_HIGH;
    Delay(5);
	SCL_LOW;
}					 				     

/**************************??��?********************************************
*��?�쫬:		void IIC_Send_Byte(u8 txd)
*�\�@�@��:	    IIC?�e�@?�r?
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
    SDA_OUT(); 	    
    SCL_LOW;//�ԧC???�l?�u??
    for(t=0;t<8;t++){              
        if((txd&0x80)>>7){
            SDA_HIGH;
        }else{
            SDA_LOW;        
        }
        txd<<=1; 	  
        
		Delay(2);   
		SCL_HIGH;
		Delay(5);
		SCL_LOW;	
		Delay(3);
    }	 
} 	 

/**************************??��?********************************************
*��?�쫬:		u8 IIC_Read_Byte(unsigned char ack)
*�\�@�@��:	    //?1?�r?�Aack=1?�A?�eACK�Aack=0�A?�enACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA?�m??�J
    for(i=0;i<8;i++ )
	{
        SCL_LOW; 
        
		Delay(5);
		SCL_HIGH;
        receive<<=1;
        if(SDA_READ)receive++;   
		
		Delay(5); 
    }					 
    if (ack)
        IIC_Ack(); //?�eACK 
    else
        IIC_NAck();//?�enACK  
    return receive;
}

/**************************??��?********************************************
*��?�쫬:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*�\�@�@��:	    ?�����w?? ���w�H�s���� length?��
?�J	dev  ��???�a�}
reg	  �H�s���a�}
length �n?���r??
*data  ?�X��?�u?�n�s�񪺫�?
��^   ?�X?���r??�q
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //?�e?�R�O
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //?�e�a�}
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //?�J�����Ҧ�	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
        
        if(count!=length-1)data[count]=IIC_Read_Byte(1);  //?ACK��??�u
        else  data[count]=IIC_Read_Byte(0);	 //�̦Z�@?�r?NACK
	}
    IIC_Stop();//?�ͤ@?����?��
    return count;
}

/**************************??��?********************************************
*��?�쫬:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*�\�@�@��:	    ?�h?�r??�J���w?? ���w�H�s��
?�J	dev  ��???�a�}
reg	  �H�s���a�}
length �n?���r??
*data  ?�n?��?�u�����a�}
��^   ��^�O�_���\
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
    
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //?�e?�R�O
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //?�e�a�}
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
    }
	IIC_Stop();//?�ͤ@?����?��
    
    return 1; //status == 0;
	
}