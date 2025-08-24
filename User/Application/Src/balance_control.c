//
// Created by Jelly on 2025/8/23.
//

#include "balance_control.h"

static PID_Info_TypeDef pid_angle_outer; // 外环：角度环
static PID_Info_TypeDef pid_rate_inner;  // 内环：角速度环

static float angle_pid_param[PID_PARAMETER_NUM] = {20.0f, 0.0f, 0.0f, 0, 0, 1.5f, 0}; // P, I, D, IMax, PMax, OutputMax
static float rate_pid_param[PID_PARAMETER_NUM]  = {0.8f, 0.0f, 0.05f, 0, 0, 0.78f, 0}; // P, I, D, IMax, PMax, OutputMax (输出最大为±45度=0.78rad)

void Balance_Controller_Init(void)
{
    // 初始化外环 (角度环) - 主要靠P
    PID_Init(&pid_angle_outer, PID_POSITION, angle_pid_param);

    // 初始化内环 (角速度环) - P和D都非常重要
    PID_Init(&pid_rate_inner, PID_POSITION, rate_pid_param);
}

float Balance_Controller_Update(float current_roll_angle, float current_roll_rate)
{
    // 1. --- 执行外环计算 ---
    //    目标是保持角度为0
    //    输出是一个期望的角速度 (rad/s)
    float target_roll_rate = PID_Calculate(&pid_angle_outer, 0.0f, current_roll_angle);

    // 2. --- 执行内环计算 ---
    //    目标是追踪外环给出的期望角速度
    //    输出是最终的转向角度 (rad)
    float target_steer_angle = PID_Calculate(&pid_rate_inner, target_roll_rate, current_roll_rate);

    return target_steer_angle;
}