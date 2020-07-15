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

extern _Time_test run_start; //�ۤv�諸
extern _Time_test run_stop; // �ۤv�諸

#endif