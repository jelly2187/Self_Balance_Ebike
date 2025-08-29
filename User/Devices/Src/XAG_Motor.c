//
// Created by Jelly on 2025/8/28.
//

#include "XAG_Motor.h"
#include "tim.h" //
// PWM 18KHZ

/*
* 极飞A25电机，
* 极飞P40动力电机/A25电机，
* 定子尺寸80x25 毫米
* KV 值：85 转速／伏
* 最大拉力（单电机）20千克
* 额定功率（单电机）1500瓦
* 电调VC13100/VC13180
* 最大持续工作电流100 安
* 最大工作电压56.6 伏（13S 锂聚合物电池）
*/

float xag_motor_pwm = 0;

void ESC_Init(void)
{
    // printf("ESC Initializing...\r\n");
    // HAL_Delay(2000);
    // 1. 启动PWM信号输出
    HAL_TIM_PWM_Start(&ESC_TIMER_HANDLE, ESC_TIMER_CHANNEL);

    // 2. 执行ESC安全解锁序列
    //    这是所有航模电调必需的步骤，防止上电后电机意外转动
    // __HAL_TIM_SET_COMPARE(&ESC_TIMER_HANDLE, ESC_TIMER_CHANNEL, 0);
    // HAL_Delay(2000);
    // __HAL_TIM_SET_COMPARE(&ESC_TIMER_HANDLE, ESC_TIMER_CHANNEL, 2000);
    // HAL_Delay(2000); // 持续2秒
    // // a. 发送最低油门信号 (1000μs)
    __HAL_TIM_SET_COMPARE(&ESC_TIMER_HANDLE, ESC_TIMER_CHANNEL, 1000);
    //
    // // b. 持续2-3秒，等待电调完成自检并发出确认音
    HAL_Delay(3000);


    // Unlock sequence complete. The ESC is now armed and ready.
}

void ESC_Set_Pulsewidth(uint16_t pulse_us)
{
    // 安全限幅，防止发送超出范围的脉宽
    if (pulse_us < MIN_PULSE_US) {
        pulse_us = MIN_PULSE_US;
    }
    if (pulse_us > MAX_PULSE_US) {
        pulse_us = MAX_PULSE_US;
    }
    xag_motor_pwm = pulse_us;
    // 更新TIM1_CH3的CCR寄存器值，从而改变PWM脉宽
    __HAL_TIM_SET_COMPARE(&ESC_TIMER_HANDLE, ESC_TIMER_CHANNEL, pulse_us);
}
