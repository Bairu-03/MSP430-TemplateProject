/*
 * PID.c
 *
 *  Created on: 2024年7月17日
 *      Author: Bairu
 */

#include "PID.h"

/**
 * @brief  指数加权移动平均滤波
 * @param  input 输入值
 * @param  filtered_value 上次滤波后的输入值
 * @param  alpha 滤波系数
 * @retval 滤波结果
 */
float exponential_filter(float input, float filtered_value, float alpha)
{
    return alpha * input + (1.0 - alpha) * filtered_value;
}

/**
 * @brief  PID参数初始化
 * @param  *pid PID参数结构体变量
 * @param  Kp 比例项系数
 * @param  Ki 积分项系数
 * @param  Kd 微分项系数
 * @param  target 目标值
 * @param  minIntegral 积分限幅-最小值
 * @param  maxIntegral 积分限幅-最大值
 * @param  minOutput PID输出限幅-最小值
 * @param  maxOutput PID输出限幅-最大值
 * @retval 无
 */
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float target,
              float minIntegral, float maxIntegral,
              float minOutput, float maxOutput)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = target;
    pid->minIntegral = minIntegral;
    pid->maxIntegral = maxIntegral;
    pid->minOutput = minOutput;
    pid->maxOutput = maxOutput;
    pid->last_error = 0.0;
    pid->integral = 0.0;
    pid->filtered_input = 0.0;
}

/**
 * @brief  重设PID目标值
 * @param  *pid PID参数结构体变量
 * @param  target 目标值
 * @retval 无
 */
void PID_ResetTarget(PID *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  位置式PID运算。
 *          当输入值与目标值存在差距时，PID运算结果会持续变化直至达到限幅值。
 *          当输入值与目标值一致后，PID运算结果将稳定在某一值。
 * @param  *pid PID参数结构体变量
 * @param  input 检测到的实时值
 * @retval 无
 */
float PID_Compute(PID *pid, float input)
{
    // 对输入值进行滤波
    pid->filtered_input = exponential_filter(input, pid->filtered_input, 0.2); // 0.2为滤波系数

    float error = pid->target - pid->filtered_input;    // 计算误差

    float p_term = pid->Kp * error;    // 比例项
    float i_term = pid->integral + pid->Ki * error;    // 积分项
    float d_term = pid->Kd * (error - pid->last_error);    // 微分项

    // 积分限幅
    if(i_term > pid->maxIntegral) i_term = pid->maxIntegral;
    else if(i_term < pid->minIntegral) i_term = pid->minIntegral;

    // 更新积分值和上次误差
    pid->integral = i_term;
    pid->last_error = error;

    float PID_Output = p_term + i_term + d_term;

    // PID输出限幅
    if(PID_Output > pid->maxOutput) PID_Output = pid->maxOutput;
    else if(PID_Output < pid->minOutput) PID_Output = pid->minOutput;

    return PID_Output;
}
