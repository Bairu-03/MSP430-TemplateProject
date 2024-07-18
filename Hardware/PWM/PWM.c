/*
 * PWM.c
 *
 *  Created on: 2024年7月15日
 *      Author: Bairu
 */

#include "driverlib.h"
#include "PWM.h"

uint16_t TA0_arr;

/**
 * @brief  TA0定时器PWM初始化，PWM频率 = 时钟源频率 / 分频系数 / (arr+1)
 * @param  clockSource 时钟源。
 *     @arg 有效取值:
 *      - \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK
 *      - \b TIMER_A_CLOCKSOURCE_ACLK
 *      - \b TIMER_A_CLOCKSOURCE_SMCLK
 *      - \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
 * @param  psc 时钟分频系数。
 *     @arg 有效取值:
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_1
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_2
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_3
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_4
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_5
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_6
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_7
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_8
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_10
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_12
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_14
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_16
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_20
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_24
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_28
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_32
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_40
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_48
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_56
 *      - \b TIMER_A_CLOCKSOURCE_DIVIDER_64
 * @param  arr 目标自动装载值 - 1。
 *     @arg 取值: 0 - 65535
 * @retval 无
 */
void TA0_PWM_Init(uint16_t clockSource, uint16_t psc, uint16_t arr)
{
    TA0_arr = arr;

    //复用输出：P1.2/TA0.1  P1.3/TA0.2  P1.4/TA0.3  P1.5/TA0.4
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5);

    Timer_A_initUpModeParam TAup = {0};
    TAup.clockSource = clockSource;
    TAup.clockSourceDivider = psc;
    TAup.timerPeriod = arr;
    TAup.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    TAup.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    TAup.timerClear = TIMER_A_DO_CLEAR;
    TAup.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &TAup);

    Timer_A_initCompareModeParam TAPWM = {0};
    TAPWM.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    TAPWM.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;

    // P1.2/TA0.1 PWM1
    TAPWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TAPWM.compareValue = (int)(arr * 0);
    Timer_A_initCompareMode(TIMER_A0_BASE, &TAPWM);

    // P1.3/TA0.2 PWM2
    TAPWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    TAPWM.compareValue = (int)(arr * 0);
    Timer_A_initCompareMode(TIMER_A0_BASE, &TAPWM);

    // P1.4/TA0.3 PWM3
    TAPWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TAPWM.compareValue = (int)(arr * 0);
    Timer_A_initCompareMode(TIMER_A0_BASE, &TAPWM);

    // P1.5/TA0.4 PWM4
    TAPWM.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TAPWM.compareValue = (int)(arr * 0);
    Timer_A_initCompareMode(TIMER_A0_BASE, &TAPWM);
}

/**
 * @brief  控制TA0各通道PWM占空比
 * @param  CHx 选择PWM输出通道。
 *     @arg 取值:
 *      - \b 0 : P1.2
 *      - \b 1 : P1.3
 *      - \b 2 : P1.4
 *      - \b 3 : P1.5
 * @param  Duty PWM占空比。
 *     @arg 取值: 0 - 100.0
 * @retval
 */
void TA0_PWM_Duty(uint8_t CHx, float Duty)
{
    uint16_t TA_CCRx[4] = {
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_CAPTURECOMPARE_REGISTER_4
    };

    Timer_A_setCompareValue(TIMER_A0_BASE,
                            TA_CCRx[CHx],
                            ((TA0_arr + 1) * Duty) / 100.0);
}
