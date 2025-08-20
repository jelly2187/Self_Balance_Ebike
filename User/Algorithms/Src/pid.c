//
// Created by Jelly on 2025/8/21.
//

#include "pid.h"

/**
  * @brief 初始化PID控制器参数
  * @param pid: PID控制器结构体指针
  * @param kp: 比例系数
  * @param ki: 积分系数
  * @param kd: 微分系数
  * @param output_max: 输出最大值
  * @param integral_max: 积分项最大值
  */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = output_max;
    pid->integral_max = integral_max;

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
  * @brief 计算PID控制器输出
  * @param pid: PID控制器结构体指针
  * @param target: 目标值
  * @param feedback: 反馈值
  * @retval PID计算得到的输出值
  */
float PID_Calculate(PID_Controller_t *pid, float target, float feedback)
{
    pid->target = target;
    pid->feedback = feedback;
    pid->error = pid->target - pid->feedback;

    // 积分项计算及限幅 (积分分离和抗饱和)
    pid->integral += pid->error;
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    // PID核心计算
    pid->output = pid->Kp * pid->error +
                  pid->Ki * pid->integral +
                  pid->Kd * (pid->error - pid->last_error);

    pid->last_error = pid->error;

    // 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < -pid->output_max)
    {
        pid->output = -pid->output_max;
    }

    return pid->output;
}