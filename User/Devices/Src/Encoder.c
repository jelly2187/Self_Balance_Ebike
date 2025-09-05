//
// Created by Jelly on 2025/9/4.
//

#include "Encoder.h"

#include <math.h>
#include <string.h>

// --- 编码器物理参数 ---
#define ENCODER_RESOLUTION      262144.0f // 2^18, 18位编码器
#define RAW_TO_RAD_COEFF        (2.0f * M_PI / ENCODER_RESOLUTION)
// ----------------------

Encoder_t xag_motor_encoder;

void Encoder_Init(int zero_point)
{
    xag_motor_encoder.zero_point = zero_point % (int)ENCODER_RESOLUTION;
}

/**
 * @brief 从CAN报文中解析原始角度数据
 * @param can_data 指向CAN 8字节数据负载的指针
 */
void Encoder_Parse_From_CAN(const uint8_t* can_data)
{
    // --- 1. 解析角度值 ---
    // 格式为 "前4字节为 Angle (uint32_t，小端内存拷贝)"
    // 由于STM32是小端模式，我们可以直接使用memcpy，这是最高效的方式
    memcpy(&xag_motor_encoder.raw_angle, can_data, sizeof(uint32_t));

    // --- 2. 解析错误标志 ---
    // "第5字节为 Error 标志"
    xag_motor_encoder.error_flag = can_data[4];
}

void Encoder_Update(float dt)
{
    // --- 1. 多圈角度累积 (翻译自您的C++代码) ---
    // 应用零点偏移
    int32_t current_raw_angle = (int32_t)xag_motor_encoder.raw_angle - xag_motor_encoder.zero_point;
    if (current_raw_angle < 0) {
        current_raw_angle += (int32_t)ENCODER_RESOLUTION;
    }

    // 计算与上一圈的差值，处理溢出
    int32_t diff = current_raw_angle - xag_motor_encoder.last_raw_angle;
    if (diff > (int32_t)(ENCODER_RESOLUTION / 2.0f)) {
        diff -= (int32_t)ENCODER_RESOLUTION;
    } else if (diff < -(int32_t)(ENCODER_RESOLUTION / 2.0f)) {
        diff += (int32_t)ENCODER_RESOLUTION;
    }

    // 累加差值得到总角度
    xag_motor_encoder.total_angle_raw += diff;
    xag_motor_encoder.last_raw_angle = current_raw_angle;

    // 转换为弧度
    xag_motor_encoder.total_angle_rad = (float)xag_motor_encoder.total_angle_raw * RAW_TO_RAD_COEFF;

    // --- 2. 速度解算 ---
    // 速度 = (当前总角度 - 上一次总角度) / 时间差
    xag_motor_encoder.speed_rad_per_sec = (xag_motor_encoder.total_angle_rad - xag_motor_encoder.last_total_angle_rad) / dt;

    // 更新上一次总角度，为下次计算做准备
    xag_motor_encoder.last_total_angle_rad = xag_motor_encoder.total_angle_rad;
}