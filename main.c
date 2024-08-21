/*************************************
 * MSP430F5529 DriverLib main函数模板
 *************************************/

#include "driverlib.h"
#include "System/Sys_Clock.h"

void main(void)
{
    WDT_A_hold(WDT_A_BASE);    // 关闭看门狗
    SystemClock_Init();        // 初始化系统时钟

    __bis_SR_register(GIE);    // 使能总中断

    while(1)
    {

    }
}
