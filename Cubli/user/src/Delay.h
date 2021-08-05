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

static volatile int TimingDelay;
void systick_setup(void);
void Delay(int nTime);
#ifdef __cplusplus
}
#endif  /* __cplusplus */
 
#endif	/* __DELAY_H */