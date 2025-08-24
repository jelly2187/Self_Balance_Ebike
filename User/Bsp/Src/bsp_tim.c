//
// Created by Jelly on 2025/8/22.
//

#include "bsp_tim.h"

#include "balance_control.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
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
        // dm_target_position_rad = Balance_Controller_Update(INS_Info.Angle[IMU_ANGLE_INDEX_ROLL],
        //                                         INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]);

        // 3. 将计算出的转向指令发送给DM8009电机
        //    注意：这里的速度前馈v_des给0，因为我们是纯位置控制
        // DM_Motor_Pos_Ctrl(&hfdcan2, CAN_ID, dm_target_position_rad, 0.0f);
    }
}
