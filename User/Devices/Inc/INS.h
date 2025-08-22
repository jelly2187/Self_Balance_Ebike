//
// Created by Jelly on 2025/8/22.
//

#ifndef BALANCE_EBIKE_INS_H
#define BALANCE_EBIKE_INS_H

#include "stdint.h"

extern uint16_t GYRO_CAL_SAMPLE_COUNT;   // 在1kHz下采样2000次，耗时2秒
extern float TARGET_TEMP;  // 目标预热温度
extern uint16_t cal_sample_count;

// --- 【新增】定义温度稳定判断的参数 ---
#define TEMP_STABILIZATION_TOLERANCE      0.5f   // 容差范围 (±0.5°C), 即39.5°C ~ 40.5°C
#define TEMP_STABILIZATION_DURATION_MS    3000   // 需要持续稳定的时间 (3000ms = 3秒)
// ------------------------------------------



typedef enum {
    INS_STATE_UNINIT,           // 未初始化
    INS_STATE_PREHEATING,       // IMU预热中
    INS_STATE_GYRO_CALIBRATING, // 陀螺仪零偏校准中
    INS_STATE_RUNNING           // 正常运行中
} INS_State_e;

typedef struct
{
    float Pitch_Angle;
    float Yaw_Angle;
    float Yaw_TolAngle;
    float Roll_Angle;

    float Pitch_Gyro;
    float Yaw_Gyro;
    float Roll_Gyro;

    float Angle[3];
    float Gyro[3];
    float Accel[3];

    float Last_Yaw_Angle;
    int16_t YawRoundCount;

}INS_Info_Typedef;

extern INS_Info_Typedef INS_Info;

void INS_Init(void);
void INS_Update(void);

INS_State_e INS_Get_State(void);

// --- 声明一个用于获取预热进度的函数，以便在main中打印 ---
void INS_Get_Preheating_Progress(uint32_t *stable_time_ms);
#endif //BALANCE_EBIKE_INS_H