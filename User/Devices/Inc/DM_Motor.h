//
// Created by Jelly on 2025/8/21.
//

#ifndef BALANCE_EBIKE_DM_MOTOR_H
#define BALANCE_EBIKE_DM_MOTOR_H

#include "fdcan.h"

#include "stm32h7xx_hal.h"

// // 电机物理参数限制 (来自官方手册)
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
#define T_MIN -54.0f
#define T_MAX 54.0f

#define CAN_ID 0x01
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPD_MODE			0x200
#define PSI_MODE		  	0x300

extern volatile float dm_target_position_rad;
// DM电机反馈信息结构体
typedef struct
{
    uint8_t id;
    uint8_t state;
    float position; // 实际位置 rad
    float velocity; // 实际速度 rad/s
    float torque;   // 实际扭矩 N·m

} DM_Motor_Feedback_t;

extern volatile DM_Motor_Feedback_t dm_motor_feedback[1]; // 假设只用1个DM电机

void DM_Motor_init(void);
void DM_Motor_Enable(uint16_t motor_id, uint16_t mode_id);
void DM_Motor_Disable(uint16_t motor_id, uint16_t mode_id);
uint8_t DM_FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
void DM_Motor_Pos_Ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel);
void DM_Motor_ParseFeedback(uint8_t *rx_data);
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);
void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void save_motor_data(uint16_t id, uint8_t rid);
void Set_Bicycle_Angle(float angle_rad);
#endif //BALANCE_EBIKE_DM_MOTOR_H