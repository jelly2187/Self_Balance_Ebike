//
// Created by Jelly on 2025/8/23.
//

#ifndef BALANCE_EBIKE_REMOTE_CONTROL_H
#define BALANCE_EBIKE_REMOTE_CONTROL_H

#include "main.h"
typedef struct
{
    // 来自遥控器的原始目标值
    float target_angle_rad;     // DM8009 目标角度 (弧度)
    float target_speed_ms;      // 自行车目标速度 (米/秒)

    // 系统状态
    // uint8_t system_enabled;     // 系统是否启用

} RC_Data_t;

void Remote_Control_Parse(void);
#endif //BALANCE_EBIKE_REMOTE_CONTROL_H