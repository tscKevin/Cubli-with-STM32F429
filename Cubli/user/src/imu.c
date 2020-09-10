#include "imu.h"
_Matrix Mat = {0};
_Matrix Mat_v2 = {0};
_Attitude att = {0};

#define kp 2.0f        //proportional gain governs rate of convergence to accelerometer/magnetometer 
#define ki 0.0001f     //integral gain governs rate of convergenceof gyroscope biases

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //quaternion elements representing theestimated orientation rotation angle is 0 degree 
float exInt = 0, eyInt = 0, ezInt = 0;    //scaled integral error  

_Time_test att_time;

//Gyroscope units are radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) 
{
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;
  //LED();
  if(ax*ay*az==0)
    return;
  
  //姿态解算时间检测
  time_check(&att_time);
  
  //[ax,ay,az]是机体坐标系下加速度计测得的重力向量(竖直向下)
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  
  //VectorA = MatrixC * VectorB
  //VectorA ：参考重力向量转到在机体下的值
  //MatrixC ：地理坐标系转机体坐标系的旋转矩阵  
  //VectorB ：参考重力向量（0,0,1）      
  //[vx,vy,vz]是地理坐标系重力分向量[0,0,1]经过DCM旋转矩阵(C(n->b))计算得到的机体坐标系中的重力向量(竖直向下)    
  
  vx = Mat.DCM_T[0][2];
  vy = Mat.DCM_T[1][2];
  vz = Mat.DCM_T[2][2];
  
  //机体坐标系下向量叉乘得到误差向量，误差e就是测量得到的vˉ和预测得到的 v^之间的相对旋转。这里的vˉ就是[ax,ay,az]’,v^就是[vx,vy,vz]’
  //利用这个误差来修正DCM方向余弦矩阵(修正DCM矩阵中的四元素)，这个矩阵的作用就是将b系和n正确的转化直到重合。
  //实际上这种修正方法只把b系和n系的XOY平面重合起来，对于z轴旋转的偏航，加速度计无可奈何，
  //但是，由于加速度计无法感知z轴上的旋转运动，所以还需要用地磁计来进一步补偿。
  //两个向量的叉积得到的结果是两个向量的模与他们之间夹角正弦的乘积a×v=|a||v|sinθ,
  //加速度计测量得到的重力向量和预测得到的机体重力向量已经经过单位化，因而他们的模是1，
  //也就是说它们向量的叉积结果仅与sinθ有关，当角度很小时，叉积结果可以近似于角度成正比。
  
  ex = ay*vz - az*vy;
  ey = az*vx - ax*vz;
  ez = ax*vy - ay*vx;
  
  //对误差向量进行积分
  exInt = exInt + ex*ki;
  eyInt = eyInt + ey*ki;
  ezInt = ezInt + ez*ki;
  
  //通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
  gx = gx + kp*ex + exInt;
  gy = gy + kp*ey + eyInt;
  gz = gz + kp*ez + ezInt;
  
  //一阶龙格库塔法更新四元数  
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)* att_time.delta_time_ms*0.0005f;//0.0005=0.001*1/2
  q1 = q1 + ( q0*gx + q2*gz - q3*gy)* att_time.delta_time_ms*0.0005f;
  q2 = q2 + ( q0*gy - q1*gz + q3*gx)* att_time.delta_time_ms*0.0005f;
  q3 = q3 + ( q0*gz + q1*gy - q2*gx)* att_time.delta_time_ms*0.0005f; 
  
  //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
  
  att.rol =  atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * rad_to_angle;
  att.pit =  asin(2.0f*(q0*q2 - q1*q3)) * rad_to_angle;
  //z轴角速度积分的偏航角
  //att.yaw += Mpu.deg_s.z  * att_time.delta_time_ms*0.0001f;
  att.yaw =  atan2(2.0f*(q0*q3 + q1*q2),q0*q0 + q1*q1 - q2*q2 - q3*q3) * rad_to_angle;
  
  //printf("the roll is %f\n",att.rol);
  //printf("the pit is %f\n",att.pit);
  //printf("the ex is %f\n",ex);
}

//旋转矩阵：机体坐标系 -> 地理坐标系(從目標座標軸到初始座標軸)
void rotation_matrix(void)
{
  Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
  Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
  Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);
  
  Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
  Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
  Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);
  
  Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
  Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
  Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
  
}

//旋转矩阵的转置矩阵：地理坐标系 -> 机体坐标系 (從初始座標軸到目標座標軸)
void rotation_matrix_T(void)
{
  Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
  Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);    
  Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2); 
  
  Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
  Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;  
  Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);    
  
  Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
  Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
  Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}

void rotation_matrix_v2(void){
  Mat_v2.DCM[0][0] = 0.8161f;
  Mat_v2.DCM[0][1] = -0.4086f;
  Mat_v2.DCM[0][2] = -0.4086f;
  
  Mat_v2.DCM[1][0] = 0.0f;
  Mat_v2.DCM[1][1] = 0.7071f;
  Mat_v2.DCM[1][2] = -0.7071f;
  
  Mat_v2.DCM[2][0] = 0.5779f;
  Mat_v2.DCM[2][1] = 0.5771f;
  Mat_v2.DCM[2][2] = 0.5771f;
}
void rotation_matrix_T_v2(void){
  Mat_v2.DCM[0][0] = 0.8161f;
  Mat_v2.DCM[0][1] = 0.0f;
  Mat_v2.DCM[0][2] = 0.5779f;
  
  Mat_v2.DCM[1][0] = -0.4086f;
  Mat_v2.DCM[1][1] = 0.7071f;
  Mat_v2.DCM[1][2] = 0.5771f;
  
  Mat_v2.DCM[2][0] = -0.4086f;
  Mat_v2.DCM[2][1] = -0.7071f;
  Mat_v2.DCM[2][2] = 0.5771f;
}
//矩阵更新，姿态解算时使用
void Matrix_ready(void)
{
  rotation_matrix();                      //旋转矩阵更新
  rotation_matrix_T();                    //旋转矩阵的逆矩阵更新
}

//===============================================================
//===============================================================
//===============================================================
//===============================================================
// Fast inverse square-root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float f_abs(float f)
{
  if (f >= 0.0f)
    return f;
  return -f;
}

int int_abs(int f)
{
  if (f >= 0)
    return f;
  return -f;
}

uint16_t to_limit(uint16_t thr_in,uint16_t thr_min,uint16_t thr_max)
{
  if(thr_in<thr_min)	thr_in = thr_min;
  if(thr_in>thr_max)	thr_in = thr_max;
  return thr_in;
}

float to_zero(float in_dat,float min_dat,float max_dat)
{
  if(in_dat>min_dat&&in_dat<max_dat)  
    in_dat = 0;
  return in_dat;
}
void  set_value(SI_F_XYZ *_in_data,float value)
{
  _in_data->x = value;
  _in_data->y = value;
  _in_data->z = value;
}

void _set_val(SI_F_XYZ *_out_data,SI_F_XYZ *_in_data)
{
  _out_data->x = _in_data->x;
  _out_data->y = _in_data->y;
  _out_data->z = _in_data->z;
}
void mymemcpy(void *des,void *src,u32 n)  
{  
  uint8_t *xdes=des;
  uint8_t *xsrc=src; 
  while(n--)
    *xdes++=*xsrc++;  
}