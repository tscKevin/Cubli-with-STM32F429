#include "MPU6050.h"

S16_XYZ  acc_raw = {0};                  //加速度計raw data存取
S16_XYZ  gyro_raw = {0};                 //陀螺儀raw data存取
SI_F_XYZ acc_raw_f = {0};
SI_F_XYZ gyro_raw_f = {0};
SI_F_XYZ acc_att_lpf = {0};
SI_F_XYZ gyro_lpf = {0};
//SI_F_XYZ gyro_offset = {0,10,-2} ;   //-128 33 -18     //陀螺儀校正數據存取
SI_F_XYZ gyro_offset = {0,2,0} ;   //-128 33 -18     //陀螺儀校正數據存取
_Mpu6050_data Mpu = {0};
SI_F_XYZ accel_offset={0,0,0};

void MPU6050_Init(void){
  IIC_Send(SlaveAddress,PWR_MGMT_1,0x0);//唤醒mpu
  /* when DLPF is disabled( DLPF_CFG=0 or 7)陀螺儀輸出頻率 = 8kHz; 
  when DLPFis enabled,陀螺儀輸出頻率 = 1KHz 
  fs(採樣頻率) = 陀螺儀輸出頻率 / (1 + SMPLRT_DIV)*/	
  IIC_Send(SlaveAddress,SMPLRT_DIV,0x00);//sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
  IIC_Send(SlaveAddress,CONFIG,0x03);//内部低通  acc:44hz	gyro:42hz register 26 :
  IIC_Send(SlaveAddress,GYRO_CONFIG,0x18);// gyro scale  ：+-2000deg/s REGISTER 27
  IIC_Send(SlaveAddress,ACCEL_CONFIG,0x10);// Accel scale ：+-8g (65536/16=4096 LSB/g) REGISTER 28 

  IIC_Send(SlaveAddress,XA_OFFSET_L_TC, 0x43);
  IIC_Send(SlaveAddress,XA_OFFSET_H, 0xff);
  IIC_Send(SlaveAddress,YA_OFFSET_L_TC, 0x9c);
  IIC_Send(SlaveAddress,YA_OFFSET_H, 0xfa);
  
  IIC_Send(SlaveAddress,ZA_OFFSET_L_TC, 0x67);
  IIC_Send(SlaveAddress,ZA_OFFSET_H, 0x01); //65536 or 4096
  
  IIC_Send(SlaveAddress,XG_OFFS_USRL, 0x6b);
  IIC_Send(SlaveAddress,XG_OFFS_USRH, 0x00);
  IIC_Send(SlaveAddress,YG_OFFS_USRL, 0xe2);
  IIC_Send(SlaveAddress,YG_OFFS_USRH, 0xff);
  IIC_Send(SlaveAddress,ZG_OFFS_USRL, 0x1b);
  IIC_Send(SlaveAddress,ZG_OFFS_USRH, 0x00);
  /*IIC_Send(SlaveAddress,XA_OFFSET_L_TC, 0x34);
  IIC_Send(SlaveAddress,XA_OFFSET_H, 0xf7);
  IIC_Send(SlaveAddress,YA_OFFSET_L_TC, 0x64);
  IIC_Send(SlaveAddress,YA_OFFSET_H, 0x04);
  
  IIC_Send(SlaveAddress,ZA_OFFSET_L_TC, 0x30);
  IIC_Send(SlaveAddress,ZA_OFFSET_H, 0x06); //65536 or 4096
  
  IIC_Send(SlaveAddress,XG_OFFS_USRL, 0x55);
  IIC_Send(SlaveAddress,XG_OFFS_USRH, 0x00);
  IIC_Send(SlaveAddress,YG_OFFS_USRL, 0xf0);
  IIC_Send(SlaveAddress,YG_OFFS_USRH, 0xff);
  IIC_Send(SlaveAddress,ZG_OFFS_USRL, 0x0c);
  IIC_Send(SlaveAddress,ZG_OFFS_USRH, 0x00);*/
}


//two bytes data access
static s16 get_data(u8 REG_Address){
  u16 H,L;
  H = IIC_Read(SlaveAddress,REG_Address);
  L = IIC_Read(SlaveAddress,REG_Address+1);
  return ((H<<8)|L);
}
//get id
u8 get_mpu_id(void){
  u8 mpu_id=0;
  mpu_id = IIC_Read(SlaveAddress,WHO_AM_I);
  printf("mpu id is %x\n",mpu_id);
  return mpu_id;
}
/*==================================
存取加速度計三軸raw data
==================================*/
void get_acc_raw(void){
//  accel_offset.x = get_data(0x06);
//  accel_offset.y = get_data(0x08);
  acc_raw.x = get_data(ACCEL_XOUT_H); // get acceleration x data 
  acc_raw.y = get_data(ACCEL_YOUT_H);
  acc_raw.z = get_data(ACCEL_ZOUT_H); 
  
  acc_raw_f.x = (float)acc_raw.x;
  acc_raw_f.y = (float)acc_raw.y;
  acc_raw_f.z = (float)acc_raw.z;
  //printf(" x raw data is %f\n",acc_raw_f.x);
}

_Butterworth_parameter gyro_30hz_parameter =
{
  //200hz---30hz
  1,  -0.7477891782585,    0.272214937925,
  0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   gyro_butter_data[3];
//存取陀螺儀三軸raw data
void get_gyro_raw(void)
{
  gyro_raw.x = get_data(GYRO_XOUT_H) + gyro_offset.x;
  gyro_raw.y = get_data(GYRO_YOUT_H) + gyro_offset.y;
  gyro_raw.z = get_data(GYRO_ZOUT_H) + gyro_offset.z;        
  
  gyro_raw_f.x = (float)butterworth_lpf(((float)gyro_raw.x),&gyro_butter_data[0],&gyro_30hz_parameter);
  gyro_raw_f.y = (float)butterworth_lpf(((float)gyro_raw.y),&gyro_butter_data[1],&gyro_30hz_parameter);
  gyro_raw_f.z = (float)butterworth_lpf(((float)gyro_raw.z),&gyro_butter_data[2],&gyro_30hz_parameter);
}
//raw -> deg/s
void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out)
{
  gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
  gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
  gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}
//raw -> rad/s
void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out)
{
  gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
  gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
  gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}

//求取IIR濾波係數
void get_iir_factor(float *out_factor,float Time, float Cut_Off)//30 or 25
{
  *out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) ); //2.0f tell compiler that 2.0 is a float point not a double 
}
/*==================================
加速度計
==================================*/
//IIR低通濾波器(加速度)
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor)
{
  acc_out->x = acc_out->x + lpf_factor*(acc_in->x - acc_out->x); 
  acc_out->y = acc_out->y + lpf_factor*(acc_in->y - acc_out->y); 
  acc_out->z = acc_out->z + lpf_factor*(acc_in->z - acc_out->z); 
}
//加速度計濾波參數

_Butterworth_parameter acc_5hz_parameter =
{
  //200hz---1hz 200hz = fs(Sampling rate) 1hz = (cut off frequency) 
  //  1,   -1.955578240315,   0.9565436765112,
  //  0.000241359049042, 0.000482718098084, 0.000241359049042
  //200hz---2hz = fs(Sampling rate) 2hz = (cut off frequency)
  //  1,   -1.911197067426,   0.9149758348014,
  //  0.0009446918438402,  0.00188938368768,0.0009446918438402
  //200hz---5hz
  1,                  -1.778631777825,    0.8008026466657,
  0.005542717210281,   0.01108543442056,  0.005542717210281
    //200hz---10hz
    //    1,   -1.561018075801,   0.6413515380576,
    //    0.02008336556421,  0.04016673112842,  0.02008336556421
    //200hz---15hz
    //    1,   -1.348967745253,   0.5139818942197,
    //    0.04125353724172,  0.08250707448344,  0.04125353724172
    //200hz---20hz
    //    1,    -1.14298050254,   0.4128015980962,
    //    0.06745527388907,   0.1349105477781,  0.06745527388907
    //200hz---30hz
    //    1,  -0.7477891782585,    0.272214937925,
    //    0.1311064399166,   0.2622128798333,   0.1311064399166 
};     

_Butterworth_data   acc_butter_data[3];
//加速度計巴特沃斯低通
void acc_butterworth_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
  acc_out->x = butterworth_lpf(acc_in->x,&acc_butter_data[0],&acc_5hz_parameter);
  acc_out->y = butterworth_lpf(acc_in->y,&acc_butter_data[1],&acc_5hz_parameter);
  acc_out->z = butterworth_lpf(acc_in->z,&acc_butter_data[2],&acc_5hz_parameter);
}
//原始加速度量轉為 g
void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
  acc_out->x = (float)(acc_in->x * acc_raw_to_g);
  acc_out->y = (float)(acc_in->y * acc_raw_to_g);
  acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}

//
volatile float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
void MPU6050_Get_Display(void){//MPU6050_Show_Data *data){
  //  /* 计算x, y, z 轴倾角，返回弧度值*/
  //  data->x = acos((IIC_Read_Two_Byte(SlaveAddress,ACCEL_XOUT_H) + accel_offset.x) / 4096.0);
  //  data->y = acos((IIC_Read_Two_Byte(SlaveAddress,ACCEL_YOUT_H) + accel_offset.y) / 4096.0);
  //  data->z = acos((IIC_Read_Two_Byte(SlaveAddress,ACCEL_ZOUT_H) + accel_offset.z) / 4096.0);
  //  
  //  /* 弧度值转换为角度值 */
  //  data->x = data->x * 57.29577;
  //  data->y = data->y * 57.29577;
  //  data->z = data->z * 57.29577;
  s16 data13 = get_data(GYRO_XOUT_H);
  s16 data14 = get_data(GYRO_YOUT_H);
  s16 data15 = get_data(GYRO_ZOUT_H);
  s16 data16 = get_data(ACCEL_XOUT_H);
  s16 data17 = get_data(ACCEL_YOUT_H);
  s16 data18 = get_data(ACCEL_ZOUT_H);
  ax = data16+accel_offset.x;
  ay = data17+accel_offset.y;
  az = data18+accel_offset.z;
  gx = data13+gyro_offset.x;
  gy = data14+gyro_offset.y;
  gz = data15+gyro_offset.z;
}