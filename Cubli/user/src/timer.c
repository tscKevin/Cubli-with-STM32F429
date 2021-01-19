#include "timer.h"

uint32_t running_tim_cnt = 0;

int Max_Pwm = 8300;
int flag_stop = 0;
// 2d平衡a 140 0.35 1850 0.025 350
// 2d平衡a+b 140 0 1550 0.025 500
//           140 0 1650 0.025 400
// 3d平衡a+b+c 
//             
// 110 0 1450 0.025 500 110 0 1000 0.025 300 110 0 1050 0.025 350
float Velocity_KP_a = 140;//110;//80;//140;    //140;  //300;
float Velocity_KI_a = 0.35;      //0     //0.35;
float Balance_KP_a = 980;//1322;//1200;//1250;    //1850; //2000;
float Balance_KI_a = 0;       //0;    //0.025;
float Balance_KD_a = 700;//500;//600;     //287;  //186.2;

float Velocity_KP_b = 140;//110;
float Velocity_KI_b = 0.35;
float Balance_KP_b = 980;
float Balance_KI_b = 0;
float Balance_KD_b = 700;//350;

float Velocity_KP_c = 140;//110;//140;       //140*0.7;
float Velocity_KI_c = 0.35;
float Balance_KP_c = 980;//240;       //1850*0.7;
float Balance_KI_c = 0;//0.025;
float Balance_KD_c = 700;//350;       //320*0.7;

float velocity_pwm_a = 0.0;
float velocity_pwm_b = 0.0;
float velocity_pwm_c = 0.0;

float balance_pwm_a = 0.0;
float balance_pwm_b = 0.0;
float balance_pwm_c = 0.0;

int PWM_a = 0;
int PWM_b = 0;
int PWM_c = 0;
int nvic_flag=0;

int encoder_a;
int encoder_b;
int encoder_c;

float rol_angle_targer=-1.0;//-2.4;//-47.4+45;
float rol_angle_offset_r=12;//-33.0+45;
float rol_angle_offset_l=-18;//-63.0+45;

float pit_angle_targer=-1.5;//-53.45;
float pit_angle_offset_r=12;//-38.45;
float pit_angle_offset_l=-18;//-68.45;

float yaw_angle_targer=0;
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
float control_velocity_x(int encoder){
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
    
    Velocity=Encoder*Velocity_KP_a + Encoder_Integral*Velocity_KI_a;      //===速度控制
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
    
    Bias =(angle-rol_angle_targer);  //=== 偏差  a 0.1 b  0.2 -45度到0度
    balance = Bias*Balance_KP_a + Bias_integral*Balance_KI_a + Gyro*Balance_KD_a;
    //  balance = Bias*Balance_KP_a                              + Gyro*Balance_KD_a;
    if(flag_stop==1) balance=0;
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
    Velocity_y=Encoder_y*Velocity_KP_b + Encoder_Integral_y*Velocity_KI_b;  //===速度控制
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
    
    Bias_y=(angle-pit_angle_targer);  //=== 偏差  a 0.1 b  0.2 
    balance_y = Bias_y*Balance_KP_b + Bias_integral_y*Balance_KI_b + Gyro*Balance_KD_b;
    if(flag_stop==1) balance_y=0;
    return balance_y;
}
/*===============================

PID-Z

===============================*/
float control_velocity_z(int encoder){
    static float Velocity_z,Encoder_Least_z,Encoder_z;
    static float Encoder_Integral_z;  
    //=============速度PI控制器=======================//	
    Encoder_Least_z=encoder;        
    Encoder_z *= 0.7;		                                                  
    Encoder_z += Encoder_Least_z*0.3;	             
    Encoder_Integral_z +=Encoder_z;                                       //===積分出位移  P比例
    
    if(Encoder_Integral_z>27000)  Encoder_Integral_z=27000;             //===積分限幅    I積分
    if(Encoder_Integral_z<-27000)	Encoder_Integral_z=-27000;            //===積分限幅    D微分
    if(flag_stop==1) Encoder_Integral_z=0,Encoder_z=0;
    
    Velocity_z=Encoder_z*Velocity_KP_c + Encoder_Integral_z*Velocity_KI_c;  //===速度控制
    if(flag_stop==1) Velocity_z=0;     
//    return 0;
    return Velocity_z;
}

float control_balance_z(float angle, float Gyro){
    static float Bias_last_z, Bias_integral_z, Bias_z;
    int balance_z;
    Bias_last_z = Bias_z; 
    Bias_integral_z += Bias_last_z;
    
    if(Bias_integral_z>4200)  	Bias_integral_z=4200;               //===積分限幅 4200
    if(Bias_integral_z<-4200)	Bias_integral_z=-4200;              //===積分限幅 4200 
    if(flag_stop==1) Bias_integral_z=0,Bias_z=0;
    
    Bias_z=(angle-yaw_angle_targer);  //=== 偏差  a 0.1 b  0.2 
    balance_z= Bias_z*Balance_KP_c + Bias_integral_z*Balance_KI_c + Gyro*Balance_KD_c;
    if(flag_stop==1) balance_z=0;
//    float Bias_z;
//    static float Last_Bias,D_Bias;
//    int balance_z;
//    if (flag_stop==1) Last_Bias=0;
//    Bias_z=Gyro;
//    D_Bias=Bias_z-Last_Bias;
//    Last_Bias=Bias_z;
//    balance_z= Bias_z*Balance_KP_c + D_Bias*Balance_KD_c;
    return balance_z;
}
/*===============================

PWM-PITCH

=================================*/
void Max_pwm_limit(int amplitude){
    if(PWM_a<-amplitude) PWM_a=-amplitude;	
    if(PWM_a>amplitude)  PWM_a=amplitude;	
    if(PWM_b<-amplitude) PWM_b=-amplitude;	
    if(PWM_b>amplitude)  PWM_b=amplitude;
    if(PWM_c<-amplitude) PWM_c=-amplitude;	
    if(PWM_c>amplitude)  PWM_c=amplitude;	
}

/*===============================

Set PWM

=================================*/
void set_pwm_a(int pwm_a){
    if (pwm_a<0){
        GPIO_SetBits(GPIOB,GPIO_Pin_13);
    }
    else{ 
        GPIO_ResetBits(GPIOB,GPIO_Pin_13);
    }
    pwm_a = int_abs(pwm_a);
    TIM8->CCR1 = pwm_a;
}
void set_pwm_b(int pwm_b){
    if (pwm_b<0){
        GPIO_SetBits(GPIOD,GPIO_Pin_8);
    }
    else{ 
        GPIO_ResetBits(GPIOD,GPIO_Pin_8);
    }
    pwm_b = int_abs(pwm_b);
    TIM8->CCR2 = pwm_b;
}
void set_pwm_c(int pwm_c){
    if (pwm_c<0){
        GPIO_SetBits(GPIOD,GPIO_Pin_9);
    }
    else{ 
        GPIO_ResetBits(GPIOD,GPIO_Pin_9);
    }
    pwm_c = int_abs(pwm_c);
    TIM8->CCR3 = pwm_c;
}
/*===============================

TIM4 interrupt - main

=================================*/

_Time_test run_start;
_Time_test run_stop;
int bb;
void ahrs(void){
    get_gyro_raw();                                                         //陀螺儀raw data
    get_deg_s(&gyro_raw_f,&Mpu.deg_s);
    get_rad_s(&gyro_raw_f,&Mpu.rad_s);
    get_acc_raw();                                                          //加速度計raw data
    acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);                //姿態演算法需要用的低通濾波器
    get_acc_g(&acc_att_lpf,&Mpu.acc_g);  
    //姿態演算
    mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z); 
    Matrix_ready(); //姿態演算矩陣更新
}
void TIM5_IRQHandler(void){
    if(TIM5->SR&0X0001){//確認更新中斷旗標為有效
        //    GPIO_ToggleBits(GPIOB,GPIO_Pin_14);
        MPU6050_Get_Display();
        running_tim_cnt++ ;
        time_check(&run_start);                                                 //timer時間測量
        
        ahrs();
        
        encoder_a=read_Encoder_a();
        encoder_b=read_Encoder_b();
        encoder_c=read_Encoder_c();
        
        velocity_pwm_a = -control_velocity_x(encoder_a);                          //velocity pid
        balance_pwm_a = -control_balance_x(att.rol,Mpu.deg_s.x);                //balance_pid
//        PWM_a = balance_pwm_a + velocity_pwm_a;
        
        velocity_pwm_b = -control_velocity_y(encoder_b);
        balance_pwm_b = -control_balance_y(att.pit,Mpu.deg_s.y);
        //PWM_b = balance_pwm_b + velocity_pwm_b;
        
        velocity_pwm_c = -control_velocity_z(encoder_c);
        balance_pwm_c = -control_balance_z(att.yaw,Mpu.deg_s.z);
        //PWM_c = balance_pwm_c + velocity_pwm_c;
        // rol => -45 , pit => -35.3        
        PWM_a=0.816137590080160f*balance_pwm_a+0.577857624383505f*balance_pwm_c+velocity_pwm_a;
        PWM_b=0.408607044761925f*balance_pwm_a+0.707106781186548f*balance_pwm_b-0.577096424326928f*balance_pwm_c+velocity_pwm_b;
        PWM_c=-0.408607044761926f*balance_pwm_a+0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c+velocity_pwm_c;
//        // rol => 45 , pit => -35.3
//        PWM_a=0.816137590080160f*balance_pwm_a+0.577857624383505f*balance_pwm_c+velocity_pwm_a;
//        PWM_b=-0.408607044761925f*balance_pwm_a+0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c+velocity_pwm_b;
//        PWM_c=-0.408607044761925f*balance_pwm_a-0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c+velocity_pwm_c;
        // rol => -45 , pit => -54.7
//        PWM_a=0.577857624f*balance_pwm_a+0.81613759f*balance_pwm_c+velocity_pwm_a;
//        PWM_b= 0.57709642f*balance_pwm_a+0.707106781f*balance_pwm_b-0.408607f*balance_pwm_c+velocity_pwm_b;
//        PWM_c=-0.57709642f*balance_pwm_a+0.707106781f*balance_pwm_b+0.408607f*balance_pwm_c+velocity_pwm_c;
        
        Max_pwm_limit(Max_Pwm);
        if (nvic_flag == 1){
            //      if(Max_Pwm++>8300)Max_Pwm=8300;//慢慢上升
            if ((att.rol>= rol_angle_offset_l) && (att.rol <= rol_angle_offset_r)/* && (att.pit>= pit_angle_offset_l) && (att.pit <=pit_angle_offset_r)*/){//balance
                set_pwm_a(PWM_a);
                set_pwm_b(PWM_b);
                set_pwm_c(PWM_c);
            }else if(((att.rol <-55) && (att.rol >=-80)) || ((att.rol > -35) && (att.rol<=-27))) {//out balance, stop wheel
                PWM_a = 0;
                PWM_b = 0;
                PWM_c = 0;
                set_pwm_a(0);
                set_pwm_b(0);
                set_pwm_c(0);
            }/*else if(att.rol<-80){//jump up
            PWM_a =-0;
            set_pwm_a(PWM_a);
        }else if(att.rol>=-13){//jump up give up
            PWM_a =0;
            set_pwm_a(PWM_a);
        }else if(att.rol<-13 && att.rol>=-18){//jump up
            PWM_a =7500;
            set_pwm_a(PWM_a);
        }else if(att.rol<-18 && att.rol>=-20){//jump up
            PWM_a =5350;
            set_pwm_a(PWM_a);
        }else if(att.rol<-20 && att.rol>=-27){//jump up
            PWM_a =3850;
            set_pwm_a(PWM_a);
        }*/
        }else{
            yaw_angle_targer=att.yaw;
            set_pwm_a(0);
            set_pwm_b(0);
            set_pwm_c(0);
        }
        Anotc_SendData();
        time_check(&run_stop);
    }
    TIM5->SR&=~(1<<0);//清除更新中斷旗標  TIM4->SR = (uint16_t)~TIM_FLAG;
}