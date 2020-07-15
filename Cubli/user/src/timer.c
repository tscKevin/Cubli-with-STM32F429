#include "timer.h"


uint32_t running_tim_cnt = 0;
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
    
    time_check(&run_stop); 
  }
  TIM4->SR&=~(1<<0);//清除更新中斷旗標  TIM4->SR = (uint16_t)~TIM_FLAG;
}