#include "jy901.h"
void IIC_GPIO_Init(void){
    GPIO_InitTypeDef iic_gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    iic_gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    iic_gpio.GPIO_Mode = GPIO_Mode_OUT;
    iic_gpio.GPIO_OType = GPIO_OType_PP;
    iic_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&iic_gpio);
	SDA_OUT();     //sda??出
	SDA_HIGH;	  	  
	SCL_HIGH;
}
//void Delay(u32 count)//用于?生400KHzIIC信?所需要的延?
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
/**************************??函?********************************************
*函?原型:		void IIC_Start(void)
*功　　能:		?生IIC起始信?
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda??出
	SDA_HIGH;	  	  
	SCL_HIGH;
	
	Delay(5);
 	SDA_LOW;//START:when CLK is high,DATA change form high to low 
	
	Delay(5);
	SCL_LOW;//?住I2C??，准??送或接收?据 
}

/**************************??函?********************************************
*函?原型:		void IIC_Stop(void)
*功　　能:	    //?生IIC停止信?
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda??出
	SCL_LOW;
	SDA_LOW;//STOP:when CLK is high DATA change form low to high
 	
    Delay(5);
	SCL_HIGH; 
	SDA_HIGH;//?送I2C???束信?
	
    Delay(5);							   	
}

/**************************??函?********************************************
*函?原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待?答信?到? 
//返回值：1，接收?答失?
//        0，接收?答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	SDA_IN();      //SDA?置??入  
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
	SCL_LOW;//???出0  
	return 0;  
} 

/**************************??函?********************************************
*函?原型:		void IIC_Ack(void)
*功　　能:	    ?生ACK?答
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

/**************************??函?********************************************
*函?原型:		void IIC_NAck(void)
*功　　能:	    ?生NACK?答
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

/**************************??函?********************************************
*函?原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC?送一?字?
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
    SDA_OUT(); 	    
    SCL_LOW;//拉低???始?据??
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

/**************************??函?********************************************
*函?原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //?1?字?，ack=1?，?送ACK，ack=0，?送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA?置??入
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
        IIC_Ack(); //?送ACK 
    else
        IIC_NAck();//?送nACK  
    return receive;
}

/**************************??函?********************************************
*函?原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    ?取指定?? 指定寄存器的 length?值
?入	dev  目???地址
reg	  寄存器地址
length 要?的字??
*data  ?出的?据?要存放的指?
返回   ?出?的字??量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //?送?命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //?送地址
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //?入接收模式	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
        
        if(count!=length-1)data[count]=IIC_Read_Byte(1);  //?ACK的??据
        else  data[count]=IIC_Read_Byte(0);	 //最后一?字?NACK
	}
    IIC_Stop();//?生一?停止?件
    return count;
}

/**************************??函?********************************************
*函?原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    ?多?字??入指定?? 指定寄存器
?入	dev  目???地址
reg	  寄存器地址
length 要?的字??
*data  ?要?的?据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
    
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //?送?命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //?送地址
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
    }
	IIC_Stop();//?生一?停止?件
    
    return 1; //status == 0;
	
}