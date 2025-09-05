//
// Created by Jelly on 2025/9/4.
//

#ifndef BALANCE_EBIKE_ENCODER_H
#define BALANCE_EBIKE_ENCODER_H


#include "stdint.h"

#define ENCODER_FEEDBACK_CAN_ID 0x018

// 编码器数据结构体
typedef struct
{
    // 原始数据和配置
    uint32_t raw_angle;         // 从CAN报文解析出的原始值
    uint8_t  error_flag;        // 从CAN报文解析出的错误标志
    int      zero_point;        // 零点偏移量

    // 多圈角度累积相关
    int64_t  total_angle_raw;   // 累积的总角度（原始值单位）
    int32_t  last_raw_angle;    // 上一次的单圈角度
    float    total_angle_rad;   // 累积的总角度（弧度单位）

    // 速度解算
    float    last_total_angle_rad; // 上一次的总角度（用于计算速度）
    float    speed_rad_per_sec;    // 解算出的角速度 (rad/s)

} Encoder_t;

// 使能其他文件可以访问编码器数据
extern Encoder_t xag_motor_encoder;

/**
 * @brief 初始化编码器模块
 * @param zero_point 编码器的零点偏移值
 */
void Encoder_Init(int zero_point);

/**
 * @brief 从CAN报文中解析原始角度数据
 * @param can_data 指向CAN数据负载的指针
 */
void Encoder_Parse_From_CAN(const uint8_t* can_data);

/**
 * @brief 周期性更新函数，计算多圈角度和瞬时速度
 * @param dt 时间步长(秒), 即调用此函数的周期
 */
void Encoder_Update(float dt);


#endif //BALANCE_EBIKE_ENCODER_H