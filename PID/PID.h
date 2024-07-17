/*
 * PID.h
 *
 *  Created on: 2024年7月17日
 *      Author: Bairu
 */

#ifndef PID_PID_H_
#define PID_PID_H_

// PID的作用概述：
// 1、P产生响应速度和力度，过小响应慢，过大会产生振荡，是I和D的基础。
// 2、I在有系统误差和外力作用时消除偏差、提高精度，同时也会增加响应速度，产生过冲，过大会产生振荡。
// 3、D抑制过冲和振荡，过小系统会过冲，过大会减慢响应速度。D的另外一个作用是抵抗外界的突发干扰，阻止系统的突变。

// PID参数结构体
typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float target; // 目标值
    float last_error; // 上一次偏差
    float integral;   // 积分值
    float filtered_input; // 滤波后的输入值
    float maxIntegral, minIntegral; //积分限幅
    float maxOutput, minOutput; //输出限幅
} PID;

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float target,
              float minIntegral, float maxIntegral,
              float minOutput, float maxOutput);
void PID_ResetTarget(PID *pid, float target);
float PID_Compute(PID *pid, float input);


#endif /* PID_PID_H_ */

/************************************************************************
 *                  PID控制例程 - 编码电机转速控制                      *
 ************************************************************************
    #include "driverlib/MSP430F5xx_6xx/driverlib.h"
    #include "System/Sys_Clock.h"
    #include "Hardware/OLED/OLED.h"
    #include "Hardware/UART/MSP430F5529_UART.h"
    #include "Hardware/Encoder/Encoder.h"
    #include "Hardware/PWM/PWM.h"
    #include "PID/PID.h"

    void main(void)
    {
        WDT_A_hold(WDT_A_BASE);
        SystemClock_Init();

        __bis_SR_register(GIE);

        OLED_Init();

        UART_Init(USCI_A1_BASE, 115200);

        TA0_PWM_Init(TIMER_A_CLOCKSOURCE_SMCLK, TIMER_A_CLOCKSOURCE_DIVIDER_40, 624);
        Encoder_Init();

        PID pid_struct;    // 定义PID参数结构体变量

        // 初始化PID参数
        float Kp = 0.24, Ki = 0.035, Kd = 0.3;
        float minIntegral = 0, maxIntegral = 100;
        float minOutput = 0, maxOutput = 100;
        float target = 100.5;
        float realvalue = 0;
        float pid_output = 0;
        PID_Init(&pid_struct, Kp, Ki, Kd, target,
                minIntegral, maxIntegral,
                minOutput, maxOutput);

        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);

        OLED_ShowString(1, 1, "Target:", 8);
        OLED_ShowString(3, 1, "Current:", 8);
        OLED_ShowString(5, 1, "pid_out:", 8);

        while(1)
        {
            // 按键修改目标值
            if(!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1))
            {
                target += 30;
                while(!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1));
            }
            if(!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1))
            {
                target -= 15;
                while(!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1));
            }
            PID_ResetTarget(&pid_struct, target);
            OLED_ShowFloat(1, 57, target, 3, 2, 8);

            // 读取电机当前转速
            realvalue = getRotatingSpeed(P20);
            OLED_ShowFloat(3, 65, realvalue, 3, 2, 8);

            // 串口打印转速数据，配合vofa+可输出转速变化曲线
            UART_printf(USCI_A1_BASE, "%.2f\n", realvalue);

            // PID运算输出 - PWM
            pid_output = PID_Compute(&pid_struct, realvalue);
            OLED_ShowFloat(5, 65, pid_output, 3, 2, 8);

            // 根据PID输出更新P1.2脚PWM占空比
            TA0_PWM_Duty(0, pid_output);
        }
    }
 ************************************************************************/
