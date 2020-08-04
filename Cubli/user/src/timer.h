#ifndef _timer_h_

#define _timer_h_

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

#include "MPU6050.h"
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

float control_velocity_z(int encoder);
float control_balance_z(float angle, float Gyro);
void Max_pwm_limit(int amplitude);
void set_pwm(int pwm_x);

extern _Time_test run_start;
extern _Time_test run_stop;
extern int encoder_x;
extern int encoder_y;
extern int encoder_z;
extern int nvic_flag;
extern int flag_stop;

extern float velocity_pwm_x;
extern float balance_pwm_x;

extern float velocity_pwm_y;
extern float balance_pwm_y;

extern float velocity_pwm_z;
extern float balance_pwm_z;

extern int PWM_x;

#endif