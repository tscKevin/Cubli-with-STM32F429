#include "usart.h"

#define usart1_tx_len 1024
#define usart1_rx_len 1024
uint8_t usart1_dma_tx_buf[usart1_tx_len];
uint8_t usart1_dma_rx_buf[usart1_rx_len];

//int fputc(int ch,FILE *f)
//{
//    USART6->SR; 
//    USART_SendData(USART6, (unsigned char) ch);
//    while(USART_GetFlagStatus(USART6,USART_FLAG_TC)!=SET);
//    return(ch);
//} 

//------------------------------usart dma------------------------------//

uint8_t usart1_sent = 1;

////DMA發送完成中斷
void DMA2_Stream7_IRQHandler(void)
{  
  if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET) //等待通道四傳輸完成
  {
    DMA_Cmd(DMA2_Stream7, DISABLE);
    DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除通道四的完成標誌
  }
  usart1_sent = 1;//重新令usart_sent的指令為1
}
//開啟一次DMA傳輸
void usart1_send(void* buf, int len){
  while(usart1_sent==0)
    //Delay(2); 
    usart1_sent = 0;
  if(len > usart1_tx_len)
  {
    len = usart1_tx_len;
  }
  mymemcpy(usart1_dma_tx_buf,buf,len); //put the buf's data into usart1_dma_tx_buf
  DMA_SetCurrDataCounter(DMA2_Stream7,len); //設置DMA內存大小
  DMA_Cmd(DMA2_Stream7, ENABLE);//開啟DMA傳輸通道 完成上述步驟即代表我們啟動一次Usart1的DMA傳輸了
} 

//int usart1_read(uint8_t** buf)
//{
//    int rx_len;
//    if(USART_GetFlagStatus(USART6,USART_FLAG_IDLE)!=RESET)
//    {
//		rx_len = USART6->SR;
//        rx_len = USART6->DR;
//		DMA_Cmd(DMA1_Channel5, DISABLE);                                    
//        DMA_ClearFlag(DMA1_FLAG_TC5);
//        rx_len = usart1_rx_len - DMA_GetCurrDataCounter(DMA1_Channel5);
//        *buf = usart1_dma_rx_buf;
//        DMA_SetCurrDataCounter(DMA1_Channel5,usart1_rx_len);                
//        DMA_Cmd(DMA1_Channel5, ENABLE);
//        
//        return rx_len;
//    }
//    return -1;
//}

//void USART6_IRQHandler(void)
//{
//	int rx_len;    
//	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
//	{
//		rx_len = USART6->SR;
//        rx_len = USART6->DR;
//		DMA_Cmd(DMA1_Channel5, DISABLE);                                    
//        USART_ClearITPendingBit(USART6, USART_IT_IDLE);                       
//        rx_len = usart1_rx_len - DMA_GetCurrDataCounter(DMA1_Channel5);
//		DMA_SetCurrDataCounter(DMA1_Channel5,usart1_rx_len);                
//		DMA_Cmd(DMA1_Channel5, ENABLE);
//	}
//}

void dma_tx_config(DMA_Stream_TypeDef* DMAy_Streamx,u32 peripheral_addr,u32 memory_addr,u16 data_length)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	                    //DMA使能     stm32f103 only have one DMA1                                                                             
  DMA_DeInit(DMAy_Streamx);                                                                                                       
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;
  DMA_InitStructure.DMA_PeripheralBaseAddr = peripheral_addr;             //外設地址 給DMA的目的初始位址 (USART的Data register)  
  DMA_InitStructure.DMA_Memory0BaseAddr =memory_addr;                      //内存地址 內存的Buf array位址(usart1_dma_tx_buf)
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                      //傳輸方向：内存到外設（外設作為目的地）
  DMA_InitStructure.DMA_BufferSize = data_length;                         //傳輸長度                      
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外設地址不變 DMA只與USART6建立聯絡     
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址自增  
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //字節傳輸  
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //字節傳輸 一次傳輸8個BITS
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //正常模式（非循環），只傳輸一次，如一次傳輸完成，下次傳輸则重新配置（關閉后再使能）其地址及長度等                                    
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;             
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  
  //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //禁止内存到内存傳輸                                     
  DMA_Init(DMAy_Streamx, &DMA_InitStructure);     
  
  DMA_ITConfig(DMAy_Streamx,DMA_IT_TC,ENABLE);
  //    DMA_ITConfig(DMA_CHx,DMA1_IT_TC4,ENABLE);                             //發送完成中斷使能
  
  //    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  //    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
  //    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
  //    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  //    NVIC_Init(&NVIC_InitStructure);	
}
//void dma_rx_config(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length)
//{
////    NVIC_InitTypeDef NVIC_InitStructure;
//    DMA_InitTypeDef DMA_InitStructure;
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//    DMA_DeInit(DMA_CHx);  
//    DMA_InitStructure.DMA_PeripheralBaseAddr = peripheral_addr;  
//    DMA_InitStructure.DMA_MemoryBaseAddr = memory_addr;  
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                    
//    DMA_InitStructure.DMA_BufferSize = data_length;  
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                      
//    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
//    DMA_Init(DMA_CHx, &DMA_InitStructure);    
//    
//    DMA_SetCurrDataCounter(DMA_CHx,usart1_rx_len);

////    DMA_ITConfig(DMA_CHx,DMA_IT_TC,ENABLE);
////    DMA_ITConfig(DMA_CHx,DMA1_IT_TC5,ENABLE); 

////    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
////    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
////    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
////    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
////    NVIC_Init(&NVIC_InitStructure);	
//    
//    DMA_Cmd(DMA_CHx, ENABLE);
//}

void usart1_init(u32 bound)
{
  GPIO_InitTypeDef   GPIO_InitStructure;   //初始化GPIO
  USART_InitTypeDef  USART_InitStructure;  //初始化Usart
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);   //使能GPIOG
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);   //使能USART2
  
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
  
  
  //USART6 Tx(PA.09) 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // represent as a usart port 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  //USART6 Rx(PA.10) 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // allow low voltage or high voltage 
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  //USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = bound; 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
  USART_InitStructure.USART_StopBits = USART_StopBits_1; 
  USART_InitStructure.USART_Parity = USART_Parity_No; 
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
  USART_Init(USART6, &USART_InitStructure);
  
  USART_Cmd(USART6, ENABLE);   
  USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE); //開啟usart的DMA傳輸功能
  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE); 
  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
  
  dma_tx_config(DMA2_Stream7,(uint32_t)&USART6->DR,(uint32_t)usart1_dma_tx_buf,usart1_tx_len); //設定外設USART6->DR而usart1_dma_tx_buf為資料初始位址 也就是我們上位機的資料
  //    dma_rx_config(DMA1_Channel5,(u32)&USART6->DR,(u32)usart1_dma_rx_buf,usart1_rx_len);
  
  //    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  //    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
  //    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
  //    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  //    NVIC_Init(&NVIC_InitStructure);	
}


/**********************************ANO**********************************/

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

uint8_t data_to_send[100];
void ANO_DT_Send_Status(void)                   
{
  u8 _cnt=0;
  vs16 _temp;
  vs32 _temp2;
  u8 sum = 0;
  u8 i;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x01;
  data_to_send[_cnt++]=0;
  
  _temp = (int)(att.rol*100);                     
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = (int)(att.pit*100);                      
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = (int)(att.yaw*100);                     
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp2 = (int32_t)(100*99.9);       		
  data_to_send[_cnt++]=BYTE3(_temp2);
  data_to_send[_cnt++]=BYTE2(_temp2);
  data_to_send[_cnt++]=BYTE1(_temp2);
  data_to_send[_cnt++]=BYTE0(_temp2);
  
  data_to_send[_cnt++]=0x01;  					
  data_to_send[_cnt++]= 1;              
  
  data_to_send[3] = _cnt-4;
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  
  usart1_send((data_to_send),_cnt);  
}


void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
  u8 _cnt=0;
  vs16 _temp;
  u8 sum = 0;
  u8 i=0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x02;
  data_to_send[_cnt++]=0;
  
  _temp = a_x;    
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = g_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = m_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  data_to_send[3] = _cnt-4;
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++] = sum;
  
  usart1_send((data_to_send),_cnt);  
}

void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
  u8 _cnt=0;
  u8 i=0;
  u8 sum = 0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x03;
  data_to_send[_cnt++]=0;
  
  data_to_send[_cnt++]=BYTE1(thr);
  data_to_send[_cnt++]=BYTE0(thr);
  
  data_to_send[_cnt++]=BYTE1(yaw);
  data_to_send[_cnt++]=BYTE0(yaw);
  
  data_to_send[_cnt++]=BYTE1(rol);
  data_to_send[_cnt++]=BYTE0(rol);
  
  data_to_send[_cnt++]=BYTE1(pit);
  data_to_send[_cnt++]=BYTE0(pit);
  
  data_to_send[_cnt++]=BYTE1(aux1);
  data_to_send[_cnt++]=BYTE0(aux1);
  
  data_to_send[_cnt++]=BYTE1(aux2);
  data_to_send[_cnt++]=BYTE0(aux2);
  
  data_to_send[_cnt++]=BYTE1(aux3);
  data_to_send[_cnt++]=BYTE0(aux3);
  
  data_to_send[_cnt++]=BYTE1(aux4);
  data_to_send[_cnt++]=BYTE0(aux4);
  
  data_to_send[_cnt++]=BYTE1(aux5);
  data_to_send[_cnt++]=BYTE0(aux5);
  
  data_to_send[_cnt++]=BYTE1(aux6);
  data_to_send[_cnt++]=BYTE0(aux6);
  
  data_to_send[3] = _cnt-4;
  
  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  
  data_to_send[_cnt++]=sum;
  
  usart1_send((data_to_send),_cnt);  
}


void ANO_DT_Send_Power(float votage, float current)
{
  u8 _cnt=0;
  u16 temp;
  
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x05;
  data_to_send[_cnt++]=0;
  
  temp = (uint16_t)100*votage;
  data_to_send[_cnt++]=BYTE1(temp);
  data_to_send[_cnt++]=BYTE0(temp);
  temp = (uint16_t)100*current;
  data_to_send[_cnt++]=BYTE1(temp);
  data_to_send[_cnt++]=BYTE0(temp);
  
  data_to_send[3] = _cnt-4;
  
  u8 sum = 0;
  for(u8 i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  
  usart1_send((data_to_send),_cnt);  
}

/* 
void ANO_DT_Send_User(    s16 user1,s16 user2,s16 user3,s16 user4,s16 user5,
float user6,float user7,float user8,float user9,float user10,
float user11,float user12,float user13,float user14,float user15)
{
u8 _cnt=0;
vs16 _temp;
float _temp_f;

u8 sum = 0;
u8 i=0;
data_to_send[_cnt++]=0xAA;
data_to_send[_cnt++]=0xAA;
data_to_send[_cnt++]=0xF1;
data_to_send[_cnt++]=0;

//1-5  int16
_temp = user1;    
data_to_send[_cnt++]=BYTE1(_temp);
data_to_send[_cnt++]=BYTE0(_temp);
_temp = user2;
data_to_send[_cnt++]=BYTE1(_temp);
data_to_send[_cnt++]=BYTE0(_temp);
_temp = user3;
data_to_send[_cnt++]=BYTE1(_temp);
data_to_send[_cnt++]=BYTE0(_temp);
_temp = user4;    
data_to_send[_cnt++]=BYTE1(_temp);
data_to_send[_cnt++]=BYTE0(_temp);
_temp = user5;
data_to_send[_cnt++]=BYTE1(_temp);
data_to_send[_cnt++]=BYTE0(_temp);

//6-10 :float
_temp_f = user6;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user7;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user8;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user9;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user10;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);


_temp_f = user11;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user12;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user13;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);
_temp_f = user14;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f); 
_temp_f = user15;
data_to_send[_cnt++]=BYTE3(_temp_f);
data_to_send[_cnt++]=BYTE2(_temp_f);
data_to_send[_cnt++]=BYTE1(_temp_f);
data_to_send[_cnt++]=BYTE0(_temp_f);

data_to_send[3] = _cnt-4;

sum = 0;
for(i=0;i<_cnt;i++)
sum += data_to_send[i];
data_to_send[_cnt++] = sum;

usart1_send((data_to_send),_cnt);  
}
*/
void ANO_DT_Send_PWM_Motor(u16 M1,u16 M2,u16 M3,u16 M4,u16 M5,u16 M6,u16 M7,u16 M8)
{
  u8 _cnt=0;
  u8 i=0;
  u8 sum = 0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x06;
  data_to_send[_cnt++]=0;
  
  data_to_send[_cnt++]=BYTE1(M1);
  data_to_send[_cnt++]=BYTE0(M1);
  
  data_to_send[_cnt++]=BYTE1(M2);
  data_to_send[_cnt++]=BYTE0(M2);
  
  data_to_send[_cnt++]=BYTE1(M3);
  data_to_send[_cnt++]=BYTE0(M3);
  
  data_to_send[_cnt++]=BYTE1(M4);
  data_to_send[_cnt++]=BYTE0(M4);
  
  data_to_send[_cnt++]=BYTE1(M5);
  data_to_send[_cnt++]=BYTE0(M5);
  
  data_to_send[_cnt++]=BYTE1(M6);
  data_to_send[_cnt++]=BYTE0(M6);
  
  data_to_send[_cnt++]=BYTE1(M7);
  data_to_send[_cnt++]=BYTE0(M7);
  
  data_to_send[_cnt++]=BYTE1(M8);
  data_to_send[_cnt++]=BYTE0(M8);
  
  data_to_send[3] = _cnt-4;
  
  for(i=0;i<_cnt;i++) 
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;
  
  usart1_send((data_to_send),_cnt);  	
}


void Anotc_SendData(void)
{
  static uint8_t ANO_debug_cnt = 0;
  //printf("usart in ");
  ANO_debug_cnt++;
  switch(ANO_debug_cnt)
  {
  case 1:
    {
      ANO_DT_Send_Status();
      break;
    }
  case 2:
    {
      ANO_DT_Send_Senser(acc_raw.x,acc_raw.y,acc_raw.z,Mpu.deg_s.x,Mpu.deg_s.y,(encoder_a*30),0,0,0);//將gyro_z改成看Encoder的值
      break;
    }
  case 3:
    {
      ANO_DT_Send_RCData(1100,1200,1300,1400,1500,1600,1700,1800,1900,1100);
      break;
    }
  case 4:
    {
      ANO_DT_Send_Power(20,1.95);
      break;
    }
  case 5:
    {
      ANO_DT_Send_PWM_Motor(PWM_a,encoder_a,0,0,1500,1600,1700,1800);
      break;
    }
    /*
  case 6:
    {
    ANO_DT_Send_User(0,0,0,(1000-Motor1.out),(1000+Motor1.out),
    key.r,main_val,Mpu.deg_s.y,att.rol,acc_raw_f.z,
    0,0,0,0,0);
    break;
  }
    */
  case 6:
    {
      ANO_debug_cnt = 0;
      break;
    }
  default: break;
  }/*
  switch(ANO_debug_cnt)
  {
case 1:
  {
  ANO_DT_Send_Status();
  break;
}
case 2:
  {
  ANO_DT_Send_Senser(acc_raw.x,acc_raw.y,acc_raw.z,gyro_raw.x,gyro_raw.y,gyro_raw.z,key.value,0,0);
  break;
}
case 3:
  {
  ANO_DT_Send_RCData(1100,1200,1300,1400,1500,1600,1700,1800,1900,1100);
  break;
}
case 4:
  {
  ANO_DT_Send_Power(13.52,57.63);
  break;
}
case 5:
  {
  ANO_DT_Send_PWM_Motor(1100,1200,1300,1400,1500,1600,1700,1800);
  break;
}
case 6:
  {
  ANO_DT_Send_User(0,0,0,(1000-Motor1.out),(1000+Motor1.out),
  key.r,main_val,Mpu.deg_s.y,att.rol,acc_raw_f.z,
  0,0,0,0,0);
  break;
}
case 7:
  {
  ANO_debug_cnt = 0;
  break;
}
  default: break;
}
  */
  
}







