#ifndef _timer_h_

#define _timer_h_

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

#include "MPU6050.h"
#include "imu.h"


typedef struct
{
  float last_time_us;
  float now_time_us;
  float delta_time_us;
  float delta_time_ms;
}_Time_test;


void time_check(_Time_test *running);
float control_velocity(int encoder);
float control_balance_x(float angle, float Gyro);
void Max_pwm_limit(int amplitude);
void set_pwm(int pwm_x);

extern _Time_test run_start;
extern _Time_test run_stop;
extern int encoder_x;
extern int encoder_y;
extern int encoder_z;
extern int nvic_flag;
extern int flag_stop;
extern float velocity_pwm_x ;
extern int balance_pwm_x ;
extern int PWM_X;

#endif