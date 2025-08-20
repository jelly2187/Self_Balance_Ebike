//
// Created by Jelly on 2025/8/21.
//

#include "bsp_fdcan.h"

/**
  * @brief FDCAN接收FIFO0回调函数
  * 当中断触发时，此函数被HAL库自动调用
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // printf("FDCAN_RxFifo0Callback\r\n");
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // 从接收FIFO0中获取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 将收到的数据交给电机解析函数处理
            DJI_Motor_ParseFeedback(RxHeader.Identifier, RxData);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 确保是我们的PID控制定时器(TIM2)触发的中断
    if (htim->Instance == TIM2)
    {
        // 1. 计算PID输出
        float pid_output = PID_Calculate(&dji_motor_speed_pid, dji_target_speed_rpm, (float)motor_feedback[0].speed_rpm);

        // 2. 将PID输出作为电流指令发送给电机
        DJI_Motor_SendCommand((int16_t)pid_output);
    }
}