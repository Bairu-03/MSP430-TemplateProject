/*
 * Encoder.c
 *
 *  Created on: 2024年7月15日
 *      Author: Bairu
 */

#include "driverlib/MSP430F5xx_6xx/driverlib.h"
#include "Encoder.h"

uint16_t P20_Count = 0;
uint16_t P22_Count = 0;
uint8_t CaptureSwitch = 0;
float P20RotatingSpeed = 0;
float P22RotatingSpeed = 0;

/**
 * @brief  初始化TA1为增计数模式，每125ms触发一次中断，计算脉冲频率
 * @param  无
 * @retval 无
 */
void Encoder_TA1_Init(void)
{
    Timer_A_initUpModeParam htim = {0};
    htim.clockSource = TIMER_A_CLOCKSOURCE_ACLK;    // 32768Hz
    htim.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    htim.timerPeriod = 64 - 1;    // (32768 / 64 / 64) = 8Hz
    htim.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    htim.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    htim.timerClear = TIMER_A_DO_CLEAR;
    htim.startTimer = true;
    Timer_A_initUpMode(TIMER_A1_BASE, &htim);
}

/**
 * @brief  初始化捕获编码器脉冲输入的IO口: P2.0, P2.2
 * @param  无
 * @retval 无
 */
void Encoder_IO_Init(void)
{
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION); //上升沿捕获中断
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
}

/**
 * @brief  初始化编码电机转速读取功能，IO口: P2.0, P2.2
 * @param  无
 * @retval 无
 */
void Encoder_Init(void)
{
    Encoder_TA1_Init();
    Encoder_IO_Init();
}

/**
 * @brief  获取编码器脉冲
 * @param  PortX  编码器脉冲输入端口
 *     @arg 有效取值:
 *      - \b P20 : P2.0脚
 *      - \b P22 : P2.2脚
 * @retval 125ms内的脉冲数
 */
float getRotatingSpeed(uint8_t PortX)
{
    if(PortX == P20) {
        return P20RotatingSpeed;
    } else if(PortX == P22) {
        return P22RotatingSpeed;
    } else {
        return 0;
    }
}

// TA1中断服务函数，计算转速
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR (void)
{
    static uint16_t P20CountBegin = 0, P20CountEnd = 0;
    static uint16_t P22CountBegin = 0, P22CountEnd = 0;
    if(!CaptureSwitch)
    {
        GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
        P20CountBegin = P20_Count;
        P22CountBegin = P22_Count;
    }
    else
    {
        GPIO_disableInterrupt(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
        P20CountEnd = P20_Count;
        P22CountEnd = P22_Count;

        // // 转速(rps)，匹配JGB37-520编码电机，减速比1:30，11线
        // P20RotatingSpeed = 8.0 * (float)(P20CountEnd - P20CountBegin) / 330.0;
        // P22RotatingSpeed = 8.0 * (float)(P22CountEnd - P22CountBegin) / 330.0;

        // 编码器计数/125ms
        P20RotatingSpeed = P20CountEnd - P20CountBegin;
        P22RotatingSpeed = P22CountEnd - P22CountBegin;

        P20_Count = 0;
        P22_Count = 0;
    }
    CaptureSwitch = !CaptureSwitch;
}

// P2.0，P2.2外部中断服务函数，每次中断计数+1
// 注意: 项目中引入本模块后，若要在其他位置使用PORT2中断，
//      则需要将此中断函数注释掉，并将此处的函数体复制到新PORT2中断函数内，否则会引发重复定义问题
#pragma vector=PORT2_VECTOR
__interrupt void Port2_interrupt(void)
{
    if(GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN0))
    {
        P20_Count++;
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0);
    }
    if(GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN2))
    {
        P22_Count++;
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);
    }
}

