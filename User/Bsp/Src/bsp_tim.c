//
// Created by Jelly on 2025/8/22.
//

#include "bsp_tim.h"

#include "DJI_Motor.h"
#include "INS.h"
#include "pid.h"
#include "stm32h7xx_hal_tim.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // 确保是我们的PID控制定时器(TIM2)触发的中断
    if (htim->Instance == TIM2) {
        // 1. 计算PID输出
        float pid_output = Motor_PID_Calculate(&dji_motor_speed_pid, dji_target_speed_rpm,
                                         (float) motor_feedback[0].speed_rpm);

        // 2. 将PID输出作为电流指令发送给电机
        DJI_Motor_SendCommand((int16_t) pid_output);
    }
    if (htim->Instance == TIM4)
    {
        INS_Update(); // 在这里以1kHz的频率调用姿态更新
        // printf("")
    }
}
