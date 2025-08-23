//
// Created by Jelly on 2025/8/23.
//

#include "bsp_mcu.h"

#include <stdbool.h>

#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "INS.h"
#include "ws2812.h"

bool System_State = true; // false: off, true: on
void System_On(void) {
    // DM_Motor_Enable(CAN_ID,POS_MODE);
    // Set_Bicycle_Speed(-15.0f); // 15 m/s
    // Set_Bicycle_Angle(0.5f);
    // WS2812_Ctrl(255,0,0);
}

void System_Off(void) {
    // DM_Motor_Disable(CAN_ID,POS_MODE);
    Set_Bicycle_Speed(0);
    Set_Bicycle_Angle(0);
    DJI_Motor_SendCommand(0); // 停止DJI电机
    // WS2812_Ctrl(0,0,0);
}