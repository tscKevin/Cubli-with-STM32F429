#include "timer.h"

uint32_t running_tim_cnt = 0;

int Max_Pwm = 8300;
int flag_stop = 0;
int nvic_flag=0;
int jump_state=0;

float rol_angle_targer=-2.0;//-1.3;
float rol_angle_offset_r=15;//-33.0+45;
float rol_angle_offset_l=-15;//-63.0+45;

float pit_angle_targer=1.4;//-45.7;////1.8;//-53.45;
float pit_angle_offset_r=15;//-38.45;
float pit_angle_offset_l=-15;//-68.45;

float yaw_angle_targer=0;
//11 0.035 874 6.7 0.5 2
//80 0.035 870 6.5 300 90
//140 0.35 1850 0.025 350 90
// 60 0.03 1250 210.7 055 09
// 60 0.03 1100 351.7 20.25 5
// 60 0.03 1200 225.7 50.5 10.9
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
float Balance_KD = 100;
float Gryo_KP = 10;
float Gryo_KD = 1;
/*------------------------not use------------------------
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
--------------------------------------------------------*/
//=================== old way ===================
//float velocity_pwm_x = 0.0;
//int balance_pwm_x = 0;
//int Velocity_KP_old = 100;
//int Velocity_KI_old = 0.65;
//int Balance_KP_old =860;
//float Balance_Ki_old = 0.035;
//float Balance_KD_old = 632;
//*===============================
//PID
//===============================*/
//float control_velocity(int encoder){
//  static float Velocity,Encoder_Least,Encoder;
//  static float Encoder_Integral;  
//  //=============�t��PI���=======================//	
//  Encoder_Least=encoder;        
//  Encoder *= 0.7;		                                                  
//  Encoder += Encoder_Least*0.3;	                                
//  Encoder_Integral +=Encoder;                                       //===�n���X�첾  P���
//  if(Encoder_Integral>27000)  	Encoder_Integral=27000;             //===�n�����T    I�n��
//  if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===�n�����T    D�L��
//  if(flag_stop==1) Encoder_Integral=0,Encoder=0; 
//  Velocity=Encoder*Velocity_KP_old+Encoder_Integral*(Velocity_KI_old);  //===�t�ױ���
//  if(flag_stop==1) Velocity=0;     
//  return Velocity;
//}
//float control_balance(float angle, float Gyro){
//  static float Bias_last, Bias_integral, Bias;
//  int balance;
//  Bias_last = Bias; 
//  Bias_integral += Bias_last;
//  if(Bias_integral>4200)  	Bias_integral=4200;               //===�n�����T 4200
//  if(Bias_integral<-4200)	Bias_integral=-4200;              //===�n�����T 4200 
//  if(flag_stop==1) Bias_integral=0,Bias=0; 
//  Bias=(angle-rol_angle_targer);  //=== ���t  a 0.1 b  0.2 
//  balance= Balance_KP_old*Bias + Gyro*Balance_KD_old + Bias_integral*Balance_Ki_old;  
//  return balance;
//}
//=================== old way ===================
float velocity_pwm_a = 0.0, velocity_pwm_b = 0.0, velocity_pwm_c = 0.0;
float balance_pwm_a = 0.0, balance_pwm_b = 0.0, balance_pwm_c = 0.0;

#define X_PARAMETER (0.5f)               
#define Y_PARAMETER (0.8660254037844f)      //sqrt(3)/2.f
#define L_PARAMETER (1.0f)
int PWM_a = 0, PWM_b = 0, PWM_c = 0;
int encoder_a, encoder_b, encoder_c;
float Encoder_X,Encoder_Y,Encoder_Z;
float Move_X,Move_Y,Move_Z;
/*===============================

���ըt�ιB��ɶ�

===============================*/
void time_check(_Time_test *running){
    running->last_time_us = running->now_time_us;
    running->now_time_us = running_tim_cnt * 5000 + TIM5->CNT;                  //�p�Ʋ֥[�ƥ[1�A?�ݭn5ms = 5000us �A�w�ɾ��w��5ms�ݭn5000���p�ơA�ҥH�@��CNT�Ȭ�1us 
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
    Encoder_Integral +=Encoder;                                     //===�n���X�첾  P���
    
    if(Encoder_Integral>27000)  Encoder_Integral=27000;             //===�n�����T    I�n��
    if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===�n�����T    D�L��
    if(flag_stop==1) Encoder=0, Encoder_Integral=0;
    
    Velocity=Encoder*Velocity_KP + Encoder_Integral*Velocity_KI;    //===�t�ױ���
    if(flag_stop==1) Velocity=0;     
    return Velocity;
}

float control_balance_x(float angle, float Gyro){
    float Bias, balance;
    Bias=(angle-rol_angle_targer);//error
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
    Encoder_Integral +=Encoder;                                     //===�n���X�첾  P���
    
    if(Encoder_Integral>27000)  Encoder_Integral=27000;             //===�n�����T    I�n��
    if(Encoder_Integral<-27000)	Encoder_Integral=-27000;            //===�n�����T    D�L��
    if(flag_stop==1) Encoder=0, Encoder_Integral=0;
    
    Velocity=Encoder*Velocity_KP + Encoder_Integral*Velocity_KI;    //===�t�ױ���
    if(flag_stop==1) Velocity=0;     
    return Velocity;
}

float control_balance_y(float angle, float Gyro){
    float Bias, balance;
    Bias=(angle-pit_angle_targer);//error
    balance=Bias*Balance_KP + Gyro*Balance_KD;
    if(flag_stop==1) balance=0;
    return balance;
}
/*===============================

PID-Z

===============================*/
float control_balance_z(float angle, float Gyro){
    static float Bias_last;
    float Bias, D_Bias;
    int balance;
    if(flag_stop==1) Bias_last=0;
    Bias = Gyro;
    D_Bias= Bias - Bias_last;
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

���V�B�ʾǤ��R

===============================*/
void Encoder_Analysis(float Va,float Vb,float Vc){ 
    Encoder_X = Va*2-Vb-Vc;
    Encoder_Y = (Vb-Vc)*1.7320508075688773;
    Encoder_Z = Va+Vb+Vc;
}
/*===============================

�f�V�B�ʾǤ��R

===============================*/
void Kinematic_Analysis(float Vx,float Vy,float Vz){
    PWM_a = Vx + L_PARAMETER*Vz;
    PWM_b = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
    PWM_c = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}

/*===============================

TIM4 interrupt - main

=================================*/
_Time_test run_start;
_Time_test run_stop;
int jump_pwm=0,jump_pwm_max=0;
void ahrs(void){
    //������raw data
    get_gyro_raw();
    get_deg_s(&gyro_raw_f,&Mpu.deg_s);
    get_rad_s(&gyro_raw_f,&Mpu.rad_s);
    //�[�t�׭praw data
    get_acc_raw();
    //���A�t��k�ݭn�Ϊ��C�q�o�i��
    acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);
    get_acc_g(&acc_att_lpf,&Mpu.acc_g);
    //���A�t��
    mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z);
    //���A�t��x�}��s
    Matrix_ready();
}

void jump_program(){
//    if(att.rol <= -10 && att.rol > -11.5){
//        // -8 ~ -11.5
//        PWM_a=-6600;
//        set_pwm(1,PWM_a,0,0);
//    }else if(att.rol <= -11.5 && att.rol > -13){ 
//        // -11.5 ~ -13
//        PWM_a=-7000;
//        set_pwm(1,PWM_a,0,0);
//    }else 
        if(att.rol < -14){
        // -13 ~
    }
}

int increase=0;
void TIM5_IRQHandler(void){
    if(TIM5->SR&0X0001){//�T�{��s���_�X�Ь�����
        MPU6050_Get_Display();
        running_tim_cnt++ ;
        time_check(&run_start);                                                 //timer�ɶ����q
        
        ahrs();
        //Kinematic
        encoder_a=-read_Encoder(1);
        encoder_b=-read_Encoder(2);
        encoder_c=-read_Encoder(3);
        //���V�B�ʾǤ��R
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
        //�f�V�B�ʾǤ��R
        Kinematic_Analysis(Move_X,Move_Y,Move_Z);
        
        /*//========== 2D old way ==========
        encoder_a=read_Encoder(1);
        velocity_pwm_x = -control_velocity(encoder_a);
        balance_pwm_x = -control_balance(att.rol,Mpu.deg_s.x);
        PWM_a = balance_pwm_x + velocity_pwm_x ;
        Max_pwm_limit(Max_Pwm);
        if (nvic_flag == 1){
            //if(Max_Pwm++>8300)Max_Pwm=8300;//�C�C�W��
            if ((att.rol>=-10.0) && (att.rol <=10.0)){//balance
//                if (increase <40){
//                    increase+=1;
//                    PWM_a=PWM_a*0.7;
//                }
                set_pwm(1,PWM_a,0,0);
            }else if(((att.rol <-10.0) && (att.rol >=-15.0)) || ((att.rol > 10.0) && (att.rol<=15))) {//out balance, stop wheel
                PWM_a = 0;
                set_pwm(1,0,0,0);
            }else if(att.rol<-15){//jump up
                PWM_a =-6100;
                set_pwm(1,PWM_a,0,0);
            }else if(att.rol>15){//jump up
                PWM_a =5050;
                set_pwm(1,PWM_a,0,0);
            }
        }else{
//            set_pwm(1,0,0,0);
        }
        // ========== old way ==========*/
        
        /*========== 3D PID ==========
        encoder_a=read_Encoder(1);
        encoder_b=read_Encoder(2);
        encoder_c=read_Encoder(3);
        velocity_pwm_a=-control_velocity_x(encoder_a);
        velocity_pwm_b=-control_velocity_y(encoder_b);
        velocity_pwm_c=-control_velocity_z(encoder_c);
        balance_pwm_a=-control_balance_x(att.rol,Mpu.deg_s.x);
        balance_pwm_b=-control_balance_y(att.pit,Mpu.deg_s.y);
        balance_pwm_c=-control_balance_z(att.yaw,Mpu.deg_s.z);
        // rol => 45 , pit => -35.3
        PWM_a=0.816137590080160f*balance_pwm_a+0.577857624383505f*balance_pwm_c;//+velocity_pwm_a;
        PWM_b=-0.408607044761925f*balance_pwm_a+0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c;//+velocity_pwm_b;
        PWM_c=-0.408607044761925f*balance_pwm_a-0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c;//+velocity_pwm_c;
        */    

        Max_pwm_limit(Max_Pwm);
        if (nvic_flag == 1){
//            if(Max_Pwm++>8300)Max_Pwm=8300;//�C�C�W��
            if ((att.rol>=-9.0) && (att.rol <=9.0)){//balance
                //3D balance
                if (att.pit>=pit_angle_targer-6 && att.pit<=pit_angle_targer+6){
                    set_pwm(123,PWM_a,PWM_b,PWM_c);
                }else{
                    //2D balance
                    set_pwm(1,1.3*PWM_a,0,0);
                }
                //2D jump to 3D
                if (jump_state==2){
                    //2D jump to 3D
                    jump_pwm+=1500;
                    if(jump_pwm>jump_pwm_max) jump_pwm=jump_pwm_max;//slowly add
                    set_pwm(123,1.3*PWM_a,-jump_pwm,jump_pwm);
                }
            }else if(((att.rol <-9.0) && (att.rol >=-15.0)) || ((att.rol > 9.0) && (att.rol<=12))) {//out balance, stop wheel
                PWM_a = 0;
                jump_pwm=0;
                set_pwm(1,0,0,0);
            }else if(att.rol<-15){//jump up
                PWM_a =-7350;
                set_pwm(1,PWM_a,0,0);
            }else if(att.rol>12 && att.rol<15){//jump up
                PWM_a =4550;
                set_pwm(1,PWM_a,0,0);
            }else if(att.rol>15){//jump up
                PWM_a =4350;
                set_pwm(1,PWM_a,0,0);
            }
//        }else{
//            PWM_a = 0, PWM_b = 0, PWM_c = 0;
//            set_pwm(123, PWM_a, PWM_b, PWM_c);
        }
        Anotc_SendData();
        time_check(&run_stop);
    }
    TIM5->SR&=~(1<<0);//�M����s���_�X��  TIM4->SR = (uint16_t)~TIM_FLAG;
}