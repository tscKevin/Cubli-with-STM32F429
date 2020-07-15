#ifndef __DELAY_H
#define __DELAY_H
 
/****************************************************************
 *                        Header include
*****************************************************************/
#include "stm32f4xx.h"
 
 
#ifdef __cplusplus
 extern "C" {
#endif  /* __cplusplus */
 
/****************************************************************
 *                     Variable declaration
*****************************************************************/
 
 
/****************************************************************
 *                     Function declaration
*****************************************************************/
//void Delay_us(uint64_t nus);
void Delay(int nTime);
//void Delay_s(uint64_t ns);
static volatile int TimingDelay;
//void Delay(int nTime);
void systick_setup(void);
 
#ifdef __cplusplus
}
#endif  /* __cplusplus */
 
#endif	/* __DELAY_H */