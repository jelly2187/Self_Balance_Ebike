//
// Created by Jelly on 2025/8/28.
//

#ifndef BALANCE_EBIKE_XAG_MOTOR_H
#define BALANCE_EBIKE_XAG_MOTOR_H


#include "stdint.h"
// 定义ESC脉宽的安全范围
#define MIN_PULSE_US 1100
#define MAX_PULSE_US 2300

// 确保定时器句柄是htim1, 通道是TIM_CHANNEL_3
// 如果您用了其他定时器或通道，请修改这里
#define ESC_TIMER_HANDLE  htim1
#define ESC_TIMER_CHANNEL TIM_CHANNEL_3
extern float xag_motor_pwm;
/**
 * @brief 初始化ESC并执行安全解锁序列
 * @note  此函数会阻塞约3秒，必须在系统初始化阶段调用
 */
void ESC_Init(void);

/**
 * @brief 设置ESC的脉宽 (油门)
 * @param pulse_us 脉宽值，单位微秒 (μs)。安全范围 1000 ~ 2000.
 */
void ESC_Set_Pulsewidth(uint16_t pulse_us);
void ESC_Calibrate_Throttle(void);



#endif //BALANCE_EBIKE_XAG_MOTOR_H