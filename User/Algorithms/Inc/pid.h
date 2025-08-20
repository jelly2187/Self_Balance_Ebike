//
// Created by Jelly on 2025/8/21.
//

#ifndef BALANCE_EBIKE_PID_H
#define BALANCE_EBIKE_PID_H

#include <stdint.h>

// PID控制器结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;       // 目标值
    float feedback;     // 反馈值

    float error;        // 当前误差
    float last_error;   // 上一次误差
    float integral;     // 积分项

    float output;       // PID计算输出
    float output_max;   // 输出限幅
    float integral_max; // 积分限幅

} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float output_max, float integral_max);
float PID_Calculate(PID_Controller_t *pid, float target, float feedback);

#endif //BALANCE_EBIKE_PID_H