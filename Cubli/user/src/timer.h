#ifndef _timer_h_

#define _timer_h_

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

#include "MPU6050.h"
#include "pwm.h"
#include "imu.h"
#include "usart.h"


typedef struct
{
        float last_time_us;
        float now_time_us;
        float delta_time_us;
        float delta_time_ms;
}_Time_test;


void time_check(_Time_test *running);
float control_velocity_x(int encoder);
float control_balance_x(float angle, float Gyro);

float control_velocity_y(int encoder);
float control_balance_y(float angle, float Gyro);

//float control_velocity_z(int encoder);
//float control_balance_z(float Gyro);
float control_balance_z(float angle, float Gyro);
void pwm_limit(int amplitude);
void set_pwm_a(int pwm_a);
void set_pwm_b(int pwm_b);
void set_pwm_c(int pwm_c);


extern float Velocity_KP_a;
//extern float Velocity_KI_a;
extern float Balance_KP_a;
//extern float Balance_KI_a;
extern float Balance_KD_a;

extern float Velocity_KP_b;
//extern float Velocity_KI_b;
extern float Balance_KP_b;
//extern float Balance_Ki_b;
extern float Balance_KD_b;

extern float Velocity_KP_c;
//extern float Velocity_KI_c;
extern float Balance_KP_c;
//extern float Balance_Ki_c;
extern float Balance_KD_c;

extern _Time_test run_start;
extern _Time_test run_stop;
extern int encoder_a;
extern int encoder_b;
extern int encoder_c;
extern int nvic_flag;
extern int flag_stop;

extern float velocity_pwm_a;
extern float balance_pwm_a;

extern float velocity_pwm_b;
extern float balance_pwm_b;

//extern float velocity_pwm_c;
extern float balance_pwm_c;

extern int PWM_a;
extern int PWM_b;
extern int PWM_c;

extern float yaw_angle_targer;

#endif