//
// Created by Jelly on 2025/8/23.
//

#include "remote_control.h"

#include "at9s_pro.h"
#include "bsp_mcu.h"
#include "config.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"

#define SBUS_MIN_VAL        200
#define SBUS_MID_VAL        976  // (200+1753)/2
#define SBUS_MAX_VAL        1753
#define SBUS_DEADZONE       50   // 摇杆中位死区

// 控制输出范围
#define MAX_STEER_ANGLE_DEG 45.0f   // DM8009 最大转向角 (度)
#define MAX_BIKE_SPEED_MS   15.0f   // 自行车最大速度 (米/秒)

static float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Remote_Control_Parse(void) {
    // CH8: SYSTEM LOCK
    if (SBUS_thoroughfare[7] == 200 && !System_State) {
        System_On();
        System_State = true;
    } else if (SBUS_thoroughfare[7] == 1800) {
        System_State = false;
        System_Off();
        // printf("SYSTEM OFF!\r\n");
    } else {
    }

    // CH1: Bicycle Direction
    dm_target_position_rad = map(SBUS_thoroughfare[0],
                                 SBUS_MIN_VAL, SBUS_MAX_VAL,
                                 -MAX_STEER_ANGLE_DEG * Angle_to_rad, MAX_STEER_ANGLE_DEG * Angle_to_rad);

    // CH3: Bicycle Speed

    uint16_t speed_sbus_val = SBUS_thoroughfare[3];

    if (abs(speed_sbus_val - 1046) < SBUS_DEADZONE) {
        Set_Bicycle_Speed(0.0f);
        // float bicycle_speed = 0.0f;
        // printf("bicycle_speed=%.2f\r\n", bicycle_speed);
    } else {
        // 您的范围是 1800-227，中点约976。我们假设227是最大反向速度，1800是最大正向速度
        // float bicycle_speed = map(speed_sbus_val,
        //                               254, 1800,
        //                               -15, 15);
        // Set_Bicycle_Speed(bicycle_speed);
        if (speed_sbus_val - 1046 > 0) {
            Set_Bicycle_Speed(-3);
        } else {
            Set_Bicycle_Speed(3);
        }

        // printf("bicycle_speed=%.2f\r\n", bicycle_speed);
    }
}
