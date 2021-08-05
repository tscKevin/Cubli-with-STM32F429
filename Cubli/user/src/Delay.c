#include "Delay.h"

//#include "systick.h"
//#include "stm32f4xx.h"
/* 系统滴答定时器 */
static volatile int TimingDelay; // 防止編譯器優化，所以加上 volatile

/*
* 滴答中斷(Tick Interrupt)，此涵式建議寫在：stm32f4xx_it.c 中
* 中斷次數=實體頻率/自訂值，自訂值由 SysTick_Config() 設定
*/
void SysTick_Handler(void)
{
    // TimingDelay 由 Delay() 給值，只要不為 0 他將會一直遞減下去...
    if (TimingDelay != 0)
    { 
        TimingDelay--;
    }
}

/*
* 延遲函數
* Delay(1000); // = 延遲一秒
*/
void Delay(int nTime)
{ 
    // 將 nTime 傳給 TimingDelay，之後讓 TimingDelay 遞減
    //systick_setup();
    TimingDelay = nTime;
    
    // 等待 SysTick_Handler() 中斷涵式
    // 把 TimingDelay 減到 0 才跳出迴圈
    // TimingDelay 非零將會一直空轉
    while(TimingDelay != 0);
}
/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void systick_setup(void)
{
    if (SysTick_Config(180000)) // 168000 也可以用 SystemCoreClock / 1000 取代
    { 
        while (1);// SysTick_Config() 的值設定太大，所以終止繼續運行 (人為疏失)
    }
}