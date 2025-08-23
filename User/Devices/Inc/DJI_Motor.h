//
// Created by Jelly on 2025/8/21.
//

#ifndef BALANCE_EBIKE_DJI_MOTOR_H
#define BALANCE_EBIKE_DJI_MOTOR_H

#include "main.h"
#include "pid.h"
#include "fdcan.h"

extern PID_Controller_t dji_motor_speed_pid;
extern volatile float dji_target_speed_rpm;

typedef struct
{
    uint16_t angle;         // 机械角度, 0-8191
    int16_t  speed_rpm;     // 转速 (RPM)
    int16_t  torque_current;// 实际转矩电流
    uint8_t  temperature;   // 温度 (°C)
} DJI_Motor_Feedback_t;

extern volatile DJI_Motor_Feedback_t motor_feedback[1];

void DJI_Motor_Init(void);
void DJI_Motor_SendCommand(int16_t motor1_current);
void DJI_Motor_ParseFeedback(uint32_t can_id, uint8_t *data);

void Set_Bicycle_Speed(float bicycle_speed_mps);
float Get_Bicycle_Speed(void);
#endif //BALANCE_EBIKE_DJI_MOTOR_H