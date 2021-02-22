#include "timer.h"

uint32_t running_tim_cnt = 0;

int Max_Pwm = 8300;
int flag_stop = 0;

//80 0.035 870 6.5 300 90
//140 0.35 1850 0.025 350
//float Velocity_KP = 11;
//float Velocity_KI = 0.035;
//float Balance_KP = 874;
////float Balance_KI = 0;
//float Balance_KD = 6.7;
//float Gryo_KP = 0.5;
//float Gryo_KD = 2;
float Velocity_KP = 80;
float Velocity_KI = 0.035;
float Balance_KP = 870;
//float Balance_KI = 0;
float Balance_KD = 6.5;
float Gryo_KP = 300;
float Gryo_KD = 150;
/*------------------------not use------------------------*/
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
/*--------------------------------------------------------*/
float velocity_pwm_a = 0.0;
float velocity_pwm_b = 0.0;
float velocity_pwm_c = 0.0;

float balance_pwm_a = 0.0;
float balance_pwm_b = 0.0;
float balance_pwm_c = 0.0;

#define X_PARAMETER (0.5f)               
#define Y_PARAMETER (0.8660254037844f)      //sqrt(3)/2.f
#define L_PARAMETER (1.0f)
int PWM_a = 0, PWM_b = 0, PWM_c = 0;
float Move_X,Move_Y,Move_Z;
int encoder_a, encoder_b, encoder_c;
float Encoder_X,Encoder_Y,Encoder_Z;

int nvic_flag=0;

float rol_angle_targer=-0.95;//-1.5;//-2.4;//-47.4+45;
float rol_angle_offset_r=15;//-33.0+45;
float rol_angle_offset_l=-15;//-63.0+45;

float pit_angle_targer=-0.8;//-1.5;//-53.45;
float pit_angle_offset_r=15;//-38.45;
float pit_angle_offset_l=-15;//-68.45;

float yaw_angle_targer=0;
/*===============================

���ըt�ιB��ɶ�

===============================*/
void time_check(_Time_test *running)
{
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
    static float angle_last, Bias_last, Bias_integral;
    /*
    Bias_last = Bias; 
    Bias_integral += Bias_last;
    
    if(Bias_integral>4200)  Bias_integral=4200;                     //===�n�����T 4200
    if(Bias_integral<-4200)	Bias_integral=-4200;                    //===�n�����T 4200 
    if(flag_stop==1) Bias_integral=0,Bias=0;
    */
    Bias =(angle-rol_angle_targer);                                 //=== ���t
    //    balance = Bias*Balance_KP + Bias_integral*Balance_KI + Gyro*Balance_KD;
    balance = Bias*Balance_KP + Gyro*Balance_KD;
//    balance = Bias*Balance_KP + (Bias-Bias_last)*Balance_KD;
    Bias_last=Bias;
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
    static float angle_last, Bias_last, Bias_integral;
    /*
    Bias_last = Bias; 
    Bias_integral += Bias_last;
    
    if(Bias_integral>4200)  Bias_integral=4200;                     //===�n�����T 4200
    if(Bias_integral<-4200)	Bias_integral=-4200;                    //===�n�����T 4200 
    if(flag_stop==1) Bias_integral=0,Bias=0;
    */
    Bias =(angle-pit_angle_targer);                                 //=== ���t
    //    balance = Bias*Balance_KP + Bias_integral*Balance_KI + Gyro*Balance_KD;
    balance = Bias*Balance_KP                              + Gyro*Balance_KD;
//    balance = Bias*Balance_KP + (Bias-Bias_last)*Balance_KD;
    Bias_last=Bias;
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
    balance= Bias*Gryo_KP + D_Bias*Gryo_KD;
    if(flag_stop==1) balance=0;
    return balance;
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
���V�B�ʾǤ��R
===============================*/
void Encoder_Analysis(float Va,float Vb,float Vc)
{ 
    Encoder_X = Va*2-Vb-Vc;
    Encoder_Y = (Vb-Vc)*1.7320508075688773;
    Encoder_Z = Va+Vb+Vc;
}
/*===============================
�f�V�B�ʾǤ��R
===============================*/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
    PWM_a = Vx + L_PARAMETER*Vz;
    PWM_b = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
    PWM_c = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}

/*===============================

TIM4 interrupt - main

=================================*/

_Time_test run_start;
_Time_test run_stop;
int bb;
void ahrs(void){
    get_gyro_raw();                                                         //������raw data
    get_deg_s(&gyro_raw_f,&Mpu.deg_s);
    get_rad_s(&gyro_raw_f,&Mpu.rad_s);
    get_acc_raw();                                                          //�[�t�׭praw data
    acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);                //���A�t��k�ݭn�Ϊ��C�q�o�i��
    get_acc_g(&acc_att_lpf,&Mpu.acc_g);  
    //���A�t��
    mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z); 
    Matrix_ready(); //���A�t��x�}��s
}
void TIM5_IRQHandler(void){
    if(TIM5->SR&0X0001){//�T�{��s���_�X�Ь�����
        //    GPIO_ToggleBits(GPIOB,GPIO_Pin_14);
        MPU6050_Get_Display();
        running_tim_cnt++ ;
        time_check(&run_start);                                                 //timer�ɶ����q
        
        ahrs();
        encoder_a=-read_Encoder_a();
        encoder_b=-read_Encoder_b();
        encoder_c=-read_Encoder_c();
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
        
        /*        // rol => -45 , pit => -35.3        
        //        PWM_a=0.816137590080160f*balance_pwm_a+0.577857624383505f*balance_pwm_c+velocity_pwm_a;
        //        PWM_b=0.408607044761925f*balance_pwm_a+0.707106781186548f*balance_pwm_b-0.577096424326928f*balance_pwm_c+velocity_pwm_b;
        //        PWM_c=-0.408607044761926f*balance_pwm_a+0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c+velocity_pwm_c;
        //        // rol => 45 , pit => -35.3
        //        PWM_a=0.816137590080160f*balance_pwm_a+0.577857624383505f*balance_pwm_c+velocity_pwm_a;
        //        PWM_b=-0.408607044761925f*balance_pwm_a+0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c+velocity_pwm_b;
        //        PWM_c=-0.408607044761925f*balance_pwm_a-0.707106781186548f*balance_pwm_b+0.577096424326928f*balance_pwm_c+velocity_pwm_c;
        // rol => -45 , pit => -54.7
        //        PWM_a=0.577857624f*balance_pwm_a+0.81613759f*balance_pwm_c+velocity_pwm_a;
        //        PWM_b= 0.57709642f*balance_pwm_a+0.707106781f*balance_pwm_b-0.408607f*balance_pwm_c+velocity_pwm_b;
        //        PWM_c=-0.57709642f*balance_pwm_a+0.707106781f*balance_pwm_b+0.408607f*balance_pwm_c+velocity_pwm_c;
        */      
        Max_pwm_limit(Max_Pwm);
        if (nvic_flag == 1){
            //      if(Max_Pwm++>8300)Max_Pwm=8300;//�C�C�W��
            if ((att.rol>= rol_angle_offset_l) && (att.rol <= rol_angle_offset_r)/* && (att.pit>= pit_angle_offset_l) && (att.pit <=pit_angle_offset_r)*/){//balance
                set_pwm_a(1.3*PWM_a);
//                set_pwm_a(PWM_a);
//                set_pwm_b(PWM_b);
//                set_pwm_c(-PWM_c);
            }else if((att.rol <rol_angle_offset_l) || (att.rol >rol_angle_offset_r)) {//out balance, stop wheel
                nvic_flag = 0;
                flag_stop = 1;
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
//            yaw_angle_targer=att.yaw;
            set_pwm_a(0);
            set_pwm_b(0);
            set_pwm_c(0);
        }
        Anotc_SendData();
        time_check(&run_stop);
    }
    TIM5->SR&=~(1<<0);//�M����s���_�X��  TIM4->SR = (uint16_t)~TIM_FLAG;
}