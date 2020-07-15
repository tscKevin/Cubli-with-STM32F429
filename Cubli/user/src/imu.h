#ifndef _imu_h_
#define _imu_h_

/****************************************************************
*                        Header include
*****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "mpu6050.h"
#include "timer.h"
#include <math.h>
/****************************************************************/
typedef struct 
{
    float ax;
    float ay;
    float az;
    
    float gx;
    float gy;
    float gz;
    
    float pit;//x
    float rol;//y
    float yaw;//z
}_Attitude;


typedef struct
{
    float DCM[3][3];        //机体坐标系 -> 地理坐标系
    float DCM_T[3][3];      //地理坐标系 -> 机体坐标系
}_Matrix;

extern _Matrix Mat;
extern _Attitude att;

//imath
float invSqrt(float x);
float fast_atan2(float y, float x) ;
uint16_t to_limit(uint16_t thr_in,uint16_t thr_min,uint16_t thr_max);
float to_zero(float in_dat,float min_dat,float max_dat);
float f_abs(float f);
int int_abs(int f);
//imu
void mymemcpy(void *des,void *src,u32 n);
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) ;
void rotation_matrix(void);
void rotation_matrix_T(void);
void Matrix_ready(void);

//void MadgwickQuaternionUpdate(float gx,float gy,float gz,float ax,float ay,float az);

#endif

