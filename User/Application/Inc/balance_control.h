//
// Created by Jelly on 2025/8/23.
//

#ifndef BALANCE_EBIKE_BALANCE_CONTROL_H
#define BALANCE_EBIKE_BALANCE_CONTROL_H

#include "pid.h"
#include "INS.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} Balance_PID_FF_Gains_t;

void Balance_Controller_Init(void);
float Balance_Controller_Update(float current_roll_angle, float current_roll_rate);

void Balance_Yu_Init(void);
void Balance_Yu_SetGains(float p, float i, float d, float f);
float Balance_Yu_Update(float desired_roll_rad, float actual_roll_rad, float actual_roll_rate_rad_s, float dt);

#endif //BALANCE_EBIKE_BALANCE_CONTROL_H