#include "timer.h"

uint32_t running_tim_cnt = 0;

int Max_Pwm = 8300;
int flag_stop = 0 ;
int Velocity_KP = 100;//120; //155 100 300 150  150 
int Velocity_KI = 65;//80; //80 200 200   80   40
int Balance_KP =860;//890;//852;// 1700  2000 501 401 401     401  410 430(4.5) 500 970 1120
float Balance_Ki = 0.035; // 0.2 
float Balance_KD = 632;//582;// 1000  1200  90 40  180 155 150 350 380  400 572 610
float velocity_pwm_x = 0.0;
int balance_pwm_x = 0;
int PWM_X = 0;
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
  running->now_time_us = running_tim_cnt * 5000 + TIM4->CNT;                  //計數累加數加1，则需要5ms = 5000us ，定時器定時5ms需要5000次計數，所以一次CNT值為1us 
  running->delta_time_us = running->now_time_us - running->last_time_us;
  running->delta_time_ms = running->delta_time_us * 0.001f;
  //printf("time interrupt is %f\n", running->delta_time_us);
}

/*===============================

PID

===============================*/
float control_velocity(int encoder){
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
  Velocity=Encoder*Velocity_KP+Encoder_Integral*(Velocity_KI/100);  //===速度控制
  if(flag_stop==1) Velocity=0;     
  return Velocity;
}

float control_balance_x(float angle, float Gyro){
  static float Bias_last, Bias_integral, Bias;
  int balance;
  Bias_last = Bias; 
  Bias_integral += Bias_last;
  if(Bias_integral>4200)  	Bias_integral=4200;               //===積分限幅 4200
  if(Bias_integral<-4200)	Bias_integral=-4200;              //===積分限幅 4200 
  if(flag_stop==1) Bias_integral=0,Bias=0; 
  Bias=(angle+0.2);  //=== 偏差  a 0.1 b  0.2 
  balance= Balance_KP*Bias + Gyro*Balance_KD/10 + Bias_integral*Balance_Ki;  
  return balance;
}
/*===============================

PWM

=================================*/
void Max_pwm_limit(int amplitude){
  if(PWM_X<-amplitude) PWM_X=-amplitude;	
  if(PWM_X>amplitude)  PWM_X=amplitude;	
}

void set_pwm(int pwm_x){
  if (pwm_x<0){
    GPIO_SetBits(GPIOG,GPIO_Pin_13);
  }
  else{ 
    //pwm_x = pwm_x+200;
    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
  }
  pwm_x = int_abs(pwm_x);
  TIM5->CCR1 = pwm_x;
}
/*===============================

TIM4 interrupt - main

=================================*/

_Time_test run_start;
_Time_test run_stop;
void TIM4_IRQHandler(void){
  if(TIM4->SR&0X0001){//確認更新中斷旗標為有效
//    GPIO_ToggleBits(GPIOG,GPIO_Pin_14);
//    MPU6050_Get_Display();
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
    
    encoder_x=read_Encoder();
    
    velocity_pwm_x = -control_velocity(encoder_x);                          //velocity pid
    balance_pwm_x = -control_balance_x(att.pit,Mpu.deg_s.x);                //balance_pid
    PWM_X =   balance_pwm_x + velocity_pwm_x ;    
    
    if (nvic_flag == 1){
      if(Max_Pwm++>8300)Max_Pwm=8300;//慢慢上升
      Max_pwm_limit(Max_Pwm);
      
      if ((att.pit>=-15.0) && (att.pit <=15.0)){//balance
        set_pwm(PWM_X);
      }else if(((att.pit <-15.0) && (att.pit >=-27.0)) || ((att.pit > 15.0) && (att.pit<=27))) {//out balance, stop wheel
        PWM_X = 0;
        set_pwm(0);
      }else if(att.pit<-27){//jump up
        PWM_X =-5500;
        set_pwm(PWM_X);
      }else if(att.pit>27){//jump up
        PWM_X =6000;
        set_pwm(PWM_X);
      }
    }
    time_check(&run_stop); 
  }
  TIM4->SR&=~(1<<0);//清除更新中斷旗標  TIM4->SR = (uint16_t)~TIM_FLAG;
}