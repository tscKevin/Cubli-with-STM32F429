#include "timer.h"

uint32_t running_tim_cnt = 0;

int Max_Pwm = 8300;
int flag_stop = 0;
int nvic_flag=0;
int jump_state=0;

float rol_angle_target=-4.5;//-1.3;
float rol_angle_offset_r=15;//-33.0+45;
float rol_angle_offset_l=-15;//-63.0+45;

float pit_angle_target=0.3;//-45.7;////1.8;//-53.45;
float pit_angle_offset_r=15;//-38.45;
float pit_angle_offset_l=-15;//-68.45;

/*
float Velocity_KP = 60;
float Velocity_KI = 0.03;
float Balance_KP = 1360;
float Balance_KD = 210.7;
float Gryo_KP = 50.5;
float Gryo_KD = 10.9;
//float Balance_KP = 1150;
////float Balance_KI = 0;
//float Balance_KD = 350.7;
//float Gryo_KP = 0.55;
//float Gryo_KD = 0.9;
*/

float Velocity_KP = 40;
float Velocity_KI = 0.03;
float Balance_KP = 900;
float Balance_KD = 155;
float Gryo_KP = 10;
float Gryo_KD = 1;
//=================== old way ===================
float velocity_pwm_x = 0.0;
int balance_pwm_x = 0;
int Velocity_KP_old = 80;
int Velocity_KI_old = 0.06;
int Balance_KP_old = 900;
float Balance_Ki_old = 0.001;
float Balance_KD_old = 185;
//*===============================
//PID
//===============================*/
float control_velocity(int encoder){
  static float Velocity,Encoder_Least,Encoder;
  static float Encoder_Integral;  
  /* 速度PI控制器 */	
  Encoder_Least=encoder;        
  Encoder *= 0.7;		                                                  
  Encoder += Encoder_Least*0.3;	                                
  Encoder_Integral +=Encoder;   /* 積分出位移  P比例 */
  if(Encoder_Integral>27000)  	Encoder_Integral=27000; /* 積分限幅    I積分 */
  if(Encoder_Integral<-27000)	Encoder_Integral=-27000;    /* 積分限幅    D微分 */
  if(flag_stop==1) Encoder_Integral=0,Encoder=0; 
  Velocity=Encoder*Velocity_KP_old+Encoder_Integral*(Velocity_KI_old);  //===速度控制
  if(flag_stop==1) Velocity=0;     
  return Velocity;
}
float control_balance(float angle, float Gyro){
  static float Bias_last, Bias_integral, Bias;
  int balance;
  Bias_last = Bias; 
  Bias_integral += Bias_last;
  if(Bias_integral>4200)  	Bias_integral=4200;               //===積分限幅 4200
  if(Bias_integral<-4200)	Bias_integral=-4200;              //===積分限幅 4200 
  if(flag_stop==1) Bias_integral=0,Bias=0; 
  Bias=(angle-rol_angle_target);  //=== 偏差  a 0.1 b  0.2 
  balance= Balance_KP_old*Bias + Bias_integral*Balance_Ki_old + Gyro*Balance_KD_old;
  return balance;
}
//=================== old way ===================
float velocity_pwm_a = 0.0, velocity_pwm_b = 0.0, velocity_pwm_c = 0.0;
float balance_pwm_a = 0.0, balance_pwm_b = 0.0, balance_pwm_c = 0.0;
int PWM_a = 0, PWM_b = 0, PWM_c = 0;
int encoder_a, encoder_b, encoder_c;

#define X_PARAMETER (0.5f)               
#define Y_PARAMETER (0.8660254037844f)      //sqrt(3)/2.f
#define L_PARAMETER (1.0f)
float Encoder_X,Encoder_Y,Encoder_Z; //順向運動學
float Move_X,Move_Y,Move_Z; //逆向運動學

/*===============================

測試系統運行時間

===============================*/
void time_check(_Time_test *running){
    running->last_time_us = running->now_time_us;
    running->now_time_us = running_tim_cnt * 5000 + TIM5->CNT;                  //計數累加數加1，?需要5ms = 5000us ，定時器定時5ms需要5000次計數，所以一次CNT值為1us 
    running->delta_time_us = running->now_time_us - running->last_time_us;
    running->delta_time_ms = running->delta_time_us * 0.001f;
    //printf("time interrupt is %f\n", running->delta_time_us);
}

/*===============================

PID-X

===============================*/
float control_velocity_x(int encoder){
    static float Velocity;
    static float Encoder, Encoder_Least, Encoder_Integral;
    
    Encoder_Least=encoder;        
    Encoder *= 0.7;		                                                  
    Encoder += Encoder_Least*0.3;	             
    Encoder_Integral +=Encoder;                                     //===積分出位移  P比例
    
    if(Encoder_Integral>27000)  Encoder_Integral=27000;             //===積分限幅    I積分
    if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===積分限幅    D微分
    if(flag_stop==1) Encoder=0, Encoder_Integral=0;
    
    Velocity=Encoder*Velocity_KP + Encoder_Integral*Velocity_KI;    //===速度控制
    if(flag_stop==1) Velocity=0;     
    return Velocity;
}

float control_balance_x(float angle, float Gyro){
    float Bias, balance;
    Bias=(angle-rol_angle_target);//error
    balance=Bias*Balance_KP + Gyro*Balance_KD;
    if(flag_stop==1) balance=0;
    return balance;
}

/*===============================

PID-Y

===============================*/
float control_velocity_y(int encoder){
    static float Velocity;
    static float Encoder, Encoder_Least, Encoder_Integral;
    
    Encoder_Least=encoder;        
    Encoder *= 0.7;		                                                  
    Encoder += Encoder_Least*0.3;	             
    Encoder_Integral +=Encoder;                                     //===積分出位移  P比例
    
    if(Encoder_Integral>27000)  Encoder_Integral=27000;             //===積分限幅    I積分
    if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===積分限幅    D微分
    if(flag_stop==1) Encoder=0, Encoder_Integral=0;
    
    Velocity=Encoder*Velocity_KP + Encoder_Integral*Velocity_KI;    //===速度控制
    if(flag_stop==1) Velocity=0;     
    return Velocity;
}

float control_balance_y(float angle, float Gyro){
    float Bias, balance;
    Bias=(angle-pit_angle_target);//error
    balance=Bias*Balance_KP + Gyro*Balance_KD;
    if(flag_stop==1) balance=0;
    return balance;
}

/*===============================

PID-Z

===============================*/
int offset_yaw=0;
float control_balance_z(float angle, float Gyro){
    static float Bias_last;
    float Bias, D_Bias;
    int balance;
    if(flag_stop==1) Bias_last=0;
    Bias = Gyro;
    D_Bias= Bias - Bias_last + offset_yaw;
    Bias_last=Bias;
    balance=Bias*Gryo_KP + D_Bias*Gryo_KD;
    if(flag_stop==1) balance=0;
    return balance;
}

/*===============================

PWM-max-limit
pwm > 8300, then pwm = 8300 and pwm < -8300, then pwm = -8300

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
motor=123, all motor change pwm
motor=1, motor a change pwm
motor=2, motor b change pwm
motor=3, motor c change pwm
=================================*/
void set_pwm(int motor, int pwm_a, int pwm_b, int pwm_c){
    if(motor==123 || motor==1){
        if (pwm_a<0){
            GPIO_SetBits(GPIOB,GPIO_Pin_13);
        }else{ 
            GPIO_ResetBits(GPIOB,GPIO_Pin_13);
        }
        pwm_a = int_abs(pwm_a);
        TIM8->CCR1 = pwm_a;
    }
    if(motor==123 || motor==2){
        if (pwm_b<0){
            GPIO_SetBits(GPIOD,GPIO_Pin_8);
        }else{ 
            GPIO_ResetBits(GPIOD,GPIO_Pin_8);
        }
        pwm_b = int_abs(pwm_b);
        TIM8->CCR2 = pwm_b;
    }
    if(motor==123 || motor==3){
        if (pwm_c<0){
            GPIO_SetBits(GPIOD,GPIO_Pin_9);
        }else{ 
            GPIO_ResetBits(GPIOD,GPIO_Pin_9);
        }
        pwm_c = int_abs(pwm_c);
        TIM8->CCR3 = pwm_c;
    }
}

/*===============================

正向運動學分析

===============================*/
void Encoder_Analysis(float Va,float Vb,float Vc){ 
    Encoder_X = Va*2-Vb-Vc;
    Encoder_Y = (Vb-Vc)*1.7320508075688773;
    Encoder_Z = Va+Vb+Vc;
}

/*===============================

逆向運動學分析

===============================*/
void Kinematic_Analysis(float Vx,float Vy,float Vz){
    PWM_a = Vx + L_PARAMETER*Vz;
    PWM_b = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
    PWM_c = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}

void ahrs(void){
    //陀螺儀raw data
    get_gyro_raw();
    get_deg_s(&gyro_raw_f,&Mpu.deg_s);
    get_rad_s(&gyro_raw_f,&Mpu.rad_s);
    //加速度計raw data
    get_acc_raw();
    //姿態演算法需要用的低通濾波器
    acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);
    get_acc_g(&acc_att_lpf,&Mpu.acc_g);
    //姿態演算
    mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z);
    //姿態演算矩陣更新
    Matrix_ready();
}
/*===============================

TIM4 interrupt - main

=================================*/
_Time_test run_start;
_Time_test run_stop;
int jump_pwm=0,jump_pwm_max=0;

void jump_program(){
    if(((att.rol <-15.5) && (att.rol >=-18.0))){// || ((att.rol > 15.5) && (att.rol<=18))) {//out balance, stop wheel
        PWM_a = 0;
        jump_pwm=0;
        set_pwm(1,0,0,0);
    }else if(att.rol<-18 && att.rol>=-22){//jump up
        PWM_a =-2470;
        set_pwm(1,PWM_a,0,0);
    }else if(att.rol<-22 && att.rol>=-27){//jump up
        PWM_a =-5150;
        set_pwm(1,PWM_a,0,0);
    }else if(att.rol<-40){
        PWM_a =-5050;
        set_pwm(1,PWM_a,0,0);
//    }else if(att.rol>12 && att.rol<15){//jump up
//        PWM_a =8300;
//        set_pwm(1,PWM_a,0,0);
//    }else if(att.rol>15){//jump up
//        PWM_a =1300;
//        set_pwm(1,PWM_a,0,0);
    }
}

void balance_3D(){
    //正向運動學分析
    Encoder_Analysis(encoder_a,encoder_b,encoder_c);
    //balance_pid
    balance_pwm_a = -control_balance_x(att.rol,Mpu.deg_s.x);
    balance_pwm_b = control_balance_y(att.pit,Mpu.deg_s.y);
    balance_pwm_c = control_balance_z(att.yaw,Mpu.deg_s.z);
    //velocity pid
    velocity_pwm_a = control_velocity_x(Encoder_X);
    velocity_pwm_b = control_velocity_y(Encoder_Y);
    Move_X = balance_pwm_a + velocity_pwm_a;
    Move_Y = balance_pwm_b + velocity_pwm_b;
    Move_Z = -balance_pwm_c;
    //逆向運動學分析
    Kinematic_Analysis(Move_X,Move_Y,Move_Z);
    Max_pwm_limit(Max_Pwm);
}

void balance_2D(){
    Velocity_KP_old = 80;
    Velocity_KI_old = 0.006;
    Balance_KP_old = 850;//500 0 85
    Balance_Ki_old = 0.0;
    Balance_KD_old = 155;
    balance_pwm_a = -control_balance(att.rol,Mpu.deg_s.x);
    velocity_pwm_a = control_velocity(encoder_a);
    PWM_a = 1.3*(balance_pwm_a + velocity_pwm_a);
    Max_pwm_limit(Max_Pwm);
}

int angle_target_sampling_num_roll=0, angle_target_sampling_num_pitch=0, sampling_range=50; // T = 5ms * sampling_range
float angle_target_sampling_buffer_roll=0.0, avg_buffer_roll=0, avg_buffer_pitch=0, angle_target_sampling_buffer_pitch=0.0;
void auto_correct_target_angle(u8 axis){
    if(axis){//2D
        if (Mpu.deg_s.x <30 && Mpu.deg_s.x > -30){
            angle_target_sampling_buffer_roll+=att.rol;
            angle_target_sampling_num_roll+=1;
            if (angle_target_sampling_num_roll>=sampling_range){
                avg_buffer_roll = angle_target_sampling_buffer_roll/sampling_range;
                if (avg_buffer_roll < rol_angle_target) rol_angle_target-=0.025;
                if (avg_buffer_roll > rol_angle_target) rol_angle_target+=0.025;
                angle_target_sampling_buffer_roll=0;
                angle_target_sampling_num_roll=0;
            }
        }
    }else{
        //3D
        if (Mpu.deg_s.x <30 && Mpu.deg_s.x > -30){
            angle_target_sampling_buffer_roll+=att.rol;
            angle_target_sampling_num_roll+=1;
            if (angle_target_sampling_num_roll>=sampling_range){
                avg_buffer_roll = angle_target_sampling_buffer_roll/sampling_range;
                if (avg_buffer_roll < rol_angle_target) rol_angle_target-=0.025;
                if (avg_buffer_roll > rol_angle_target) rol_angle_target+=0.025;
                angle_target_sampling_buffer_roll=0;
                angle_target_sampling_num_roll=0;
            }
        }
        if (Mpu.deg_s.y <30 && Mpu.deg_s.y > -30){
            angle_target_sampling_buffer_pitch+=att.pit;
            angle_target_sampling_num_pitch+=1;
            if (angle_target_sampling_num_pitch>=sampling_range){
                avg_buffer_pitch=angle_target_sampling_buffer_pitch/sampling_range;
                if (avg_buffer_pitch < pit_angle_target) pit_angle_target-=0.025;
                if (avg_buffer_pitch > pit_angle_target) pit_angle_target+=0.025;
                angle_target_sampling_buffer_pitch=0;
                angle_target_sampling_num_pitch=0;
            }
        }
    }
}

void TIM5_IRQHandler(void){
    if(TIM5->SR&0X0001){//確認更新中斷旗標為有效
        MPU6050_Get_Display();
        running_tim_cnt++ ;
        time_check(&run_start);//timer時間測量
        ahrs();
        encoder_a=-read_Encoder(1);
        encoder_b=-read_Encoder(2);
        encoder_c=-read_Encoder(3);
        if (nvic_flag == 1){
            if ((att.rol>=-15.5) && (att.rol <=30.5)){//check 2D balance
                if (att.pit>=pit_angle_target-8 && att.pit<=pit_angle_target+8){//3D balance
                    balance_3D();
                    set_pwm(123,PWM_a,PWM_b,PWM_c);
                    auto_correct_target_angle(0);
                }else if (jump_state==2){//2D jump to 3D
                    balance_2D();
                    jump_pwm+=500;
                    if(jump_pwm>jump_pwm_max) jump_pwm=jump_pwm_max;//slowly add
                    set_pwm(123,PWM_a,-(jump_pwm+0),jump_pwm);
                }else{// keep 2D balance
                    balance_2D();
                    set_pwm(123,PWM_a,0,0);
                    auto_correct_target_angle(1);
                }
            }else{
                jump_program();
            }
        }
        Anotc_SendData();
        time_check(&run_stop);
    }
    TIM5->SR&=~(1<<0);//清除更新中斷旗標  TIM4->SR = (uint16_t)~TIM_FLAG;
}