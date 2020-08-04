#include "timer.h"

uint32_t running_tim_cnt = 0;

int Max_Pwm = 8300;
int flag_stop = 0 ;
float Velocity_KP_pit = 100;//300;
float Velocity_KI_pit = 0.35;
float Balance_KP_pit =1500;//2000;
float Balance_Ki_pit = 0.02;
float Balance_KD_pit = 155.2;

float Velocity_KP = 150;
float Velocity_KI = 0.55;
float Balance_KP =1500;
float Balance_Ki = 0.02;
float Balance_KD = 186.2;

float velocity_pwm_x = 0.0;
float balance_pwm_x = 0.0;

float velocity_pwm_y = 0.0;
float balance_pwm_y = 0.0;

float velocity_pwm_z = 0.0;
float balance_pwm_z = 0.0;
int PWM_x = 0;
int PWM_y = 0;
int PWM_z = 0;
int nvic_flag=0;

int encoder_x;
int encoder_y;
int encoder_z;
/*===============================

測試系統運行時間

===============================*/
void time_check(_Time_test *running)
{
  running->last_time_us = running->now_time_us;
  running->now_time_us = running_tim_cnt * 5000 + TIM5->CNT;                  //計數累加數加1，则需要5ms = 5000us ，定時器定時5ms需要5000次計數，所以一次CNT值為1us 
  running->delta_time_us = running->now_time_us - running->last_time_us;
  running->delta_time_ms = running->delta_time_us * 0.001f;
  //printf("time interrupt is %f\n", running->delta_time_us);
}

/*===============================

PID-X

===============================*/
  static float Velocity,Encoder_Least,Encoder;
  static float Encoder_Integral;  
float control_velocity_x(int encoder){
  //=============速度PI控制器=======================//	
  Encoder_Least=encoder;        
  Encoder *= 0.7;		                                                  
  Encoder += Encoder_Least*0.3;	             
  Encoder_Integral +=Encoder;                                       //===積分出位移  P比例
  
  if(Encoder_Integral>27000)  	Encoder_Integral=27000;             //===積分限幅    I積分
  if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===積分限幅    D微分
  if(flag_stop==1) Encoder_Integral=0,Encoder=0;
  
  Velocity=Encoder*Velocity_KP_pit + Encoder_Integral*Velocity_KI_pit;      //===速度控制
  if(flag_stop==1) Velocity=0;     
  return Velocity;
}

float control_balance_x(float angle, float Gyro){
  static float Bias_last, Bias_integral, Bias;
  float balance;
  Bias_last = Bias; 
  Bias_integral += Bias_last;
  
  if(Bias_integral>4200)  	Bias_integral=4200;               //===積分限幅 4200
  if(Bias_integral<-4200)	Bias_integral=-4200;              //===積分限幅 4200 
  if(flag_stop==1) Bias_integral=0,Bias=0;
  
  Bias=(angle+0.325);  //=== 偏差  a 0.1 b  0.2 
  balance= Bias*Balance_KP_pit + Bias_integral*Balance_Ki_pit + Gyro*Balance_KD_pit;
  return balance;
}
/*===============================

PID-Y

===============================*/
float control_velocity_y(int encoder){
  static float Velocity_y,Encoder_Least_y,Encoder_y;
  static float Encoder_Integral_y;  
  //=============速度PI控制器=======================//	
  Encoder_Least_y=encoder;        
  Encoder_y *= 0.7;		                                                  
  Encoder_y += Encoder_Least_y*0.3;	             
  Encoder_Integral_y +=Encoder_y;                                         //===積分出位移  P比例
  
  if(Encoder_Integral_y>27000) Encoder_Integral_y=27000;//===積分限幅     I積分
  if(Encoder_Integral_y<-27000)	Encoder_Integral_y=-27000;               //===積分限幅     D微分
  if(flag_stop==1) Encoder_Integral_y=0,Encoder_y=0;
  Velocity_y=Encoder_y*Velocity_KP + Encoder_Integral_y*Velocity_KI;  //===速度控制
  if(flag_stop==1) Velocity_y=0;     
  return Velocity_y;
}

float control_balance_y(float angle, float Gyro){
  static float Bias_last_y, Bias_integral_y, Bias_y;
  float balance_y;
  Bias_last_y = Bias_y; 
  Bias_integral_y += Bias_last_y;
  
  if(Bias_integral_y>4200)  	Bias_integral_y=4200;               //===積分限幅 4200
  if(Bias_integral_y<-4200)	Bias_integral_y=-4200;              //===積分限幅 4200 
  if(flag_stop==1) Bias_integral_y=0,Bias_y=0;
  
  Bias_y=(-45)-(angle+0.325);  //=== 偏差  a 0.1 b  0.2 
  balance_y = Bias_y*Balance_KP + Bias_integral_y*Balance_Ki + Gyro*Balance_KD;
  return balance_y;
}
/*===============================

PID-Z

===============================*/
float control_velocity_z(int encoder){
  static float Velocity,Encoder_Least,Encoder;
  static float Encoder_Integral;  
  //=============速度PI控制器=======================//	
  Encoder_Least=encoder;        
  Encoder *= 0.7;		                                                  
  Encoder += Encoder_Least*0.3;	             
  Encoder_Integral +=Encoder;                                       //===積分出位移  P比例
  
  if(Encoder_Integral>27000)  	Encoder_Integral=27000;             //===積分限幅    I積分
  if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===積分限幅    D微分
  if(flag_stop==1) Encoder_Integral=0,Encoder=0;
  
  Velocity=Encoder*Velocity_KP + Encoder_Integral*Velocity_KI;  //===速度控制
  if(flag_stop==1) Velocity=0;     
  return Velocity;
}

float control_balance_z(float angle, float Gyro){
  static float Bias_last, Bias_integral, Bias;
  int balance;
  Bias_last = Bias; 
  Bias_integral += Bias_last;
  
  if(Bias_integral>4200)  	Bias_integral=4200;               //===積分限幅 4200
  if(Bias_integral<-4200)	Bias_integral=-4200;              //===積分限幅 4200 
  if(flag_stop==1) Bias_integral=0,Bias=0;
  
  Bias=(angle+0.3);  //=== 偏差  a 0.1 b  0.2 
  balance= Bias*Balance_KP + Bias_integral*Balance_Ki + Gyro*Balance_KD;
  return balance;
}
/*===============================

PWM-PITCH

=================================*/
void Max_pwm_limit(int amplitude){
  if(PWM_x<-amplitude) PWM_x=-amplitude;	
  if(PWM_x>amplitude)  PWM_x=amplitude;	
  if(PWM_y<-amplitude) PWM_y=-amplitude;	
  if(PWM_y>amplitude)  PWM_y=amplitude;	
  if(PWM_z<-amplitude) PWM_z=-amplitude;	
  if(PWM_z>amplitude)  PWM_z=amplitude;	
}

void set_pwm(int pwm_x){
  if (pwm_x<0){
    GPIO_SetBits(GPIOB,GPIO_Pin_13);
  }
  else{ 
    //pwm_x = pwm_x+200;
    GPIO_ResetBits(GPIOB,GPIO_Pin_13);
  }
  pwm_x = int_abs(pwm_x);
  TIM8->CCR1 = pwm_x;
}
/*===============================

PWM-YAW

=================================*/
void set_pwm_y(int pwm_y){
  if (pwm_y<0){
    GPIO_SetBits(GPIOD,GPIO_Pin_8);
  }
  else{ 
    GPIO_ResetBits(GPIOD,GPIO_Pin_8);
  }
  pwm_y = int_abs(pwm_y);
  TIM8->CCR2 = pwm_y;
}
/*===============================

PWM-ROLL

=================================*/
void set_pwm_z(int pwm_z){
  if (pwm_z<0){
    GPIO_SetBits(GPIOD,GPIO_Pin_9);
  }
  else{ 
    GPIO_ResetBits(GPIOD,GPIO_Pin_9);
  }
  pwm_z = int_abs(pwm_z);
  TIM8->CCR3 = pwm_z;
}
/*===============================

TIM4 interrupt - main

=================================*/

_Time_test run_start;
_Time_test run_stop;
void TIM5_IRQHandler(void){
  if(TIM5->SR&0X0001){//確認更新中斷旗標為有效
//    GPIO_ToggleBits(GPIOB,GPIO_Pin_14);
    MPU6050_Get_Display();
    running_tim_cnt++ ;
    time_check(&run_start);                                                 //timer時間測量
    
    get_gyro_raw();                                                         //陀螺儀raw data
    get_deg_s(&gyro_raw_f,&Mpu.deg_s);
    get_rad_s(&gyro_raw_f,&Mpu.rad_s);
    get_acc_raw();                                                          //加速度計raw data
    acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);                //姿態演算法需要用的低通濾波器
    get_acc_g(&acc_att_lpf,&Mpu.acc_g);  
    //姿態演算
    mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z); 
    Matrix_ready();                                                         //姿態演算矩陣更新                                                       //姿態演算矩陣更新
    
    encoder_x=read_Encoder_x();
    encoder_y=read_Encoder_y();
    encoder_z=read_Encoder_z();
    
    velocity_pwm_x = -control_velocity_x(encoder_x);                          //velocity pid
    balance_pwm_x = -control_balance_x(att.pit,Mpu.deg_s.x);                //balance_pid
    PWM_x = balance_pwm_x + velocity_pwm_x;
    
    velocity_pwm_y = -control_velocity_y(encoder_y);
    balance_pwm_y = -control_balance_y(att.rol,Mpu.deg_s.y);
    PWM_y = balance_pwm_y + velocity_pwm_y;
    
    velocity_pwm_z = -control_velocity_z(encoder_z);
    balance_pwm_z = -control_balance_z(att.yaw,Mpu.deg_s.z);
    PWM_z = balance_pwm_z + velocity_pwm_z;
    
    if (nvic_flag == 1){
//      if(Max_Pwm++>8300)Max_Pwm=8300;//慢慢上升
      Max_pwm_limit(Max_Pwm);
      
      if ((att.pit>=-15) && (att.pit <=15)){//balance
        set_pwm(PWM_x);
        set_pwm_y(PWM_x);
        set_pwm_z(PWM_x);
//        int PWM_xz=(PWM_x+PWM_z);
//        if (PWM_xz >=8300) PWM_xz=8300;
//        if (PWM_xz <=-8300) PWM_xz=-8300;
//        set_pwm(PWM_xz);
//        int PWM_xy=(PWM_x+PWM_y);
//        if (PWM_xy >=8300) PWM_xy=8300;
//        if (PWM_xy <=-8300) PWM_xy=-8300;
//        set_pwm_y(PWM_xy);
//        int PWM_yz=(-PWM_y+PWM_z);
//        if (PWM_yz >=8300) PWM_yz=8300;
//        if (PWM_yz <=-8300) PWM_yz=-8300;
//        set_pwm_z(PWM_yz);
      }else if(((att.pit <-15) && (att.pit >=-27)) || ((att.pit > 15) && (att.pit<=27))) {//out balance, stop wheel
        PWM_x = 0;
        set_pwm(0);
      }/*else if(att.pit<-27){//jump up
        PWM_x =-5500;
        set_pwm(PWM_x);
      }else if(att.pit>27){//jump up
        PWM_x =6000;
        set_pwm(PWM_x);
      }*/
    }
    Anotc_SendData();
    time_check(&run_stop);
  }
  TIM5->SR&=~(1<<0);//清除更新中斷旗標  TIM4->SR = (uint16_t)~TIM_FLAG;
}