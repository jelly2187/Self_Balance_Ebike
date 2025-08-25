//
// Created by Jelly on 2025/8/23.
//

#include "balance_control.h"
#include <math.h>
#include <stdio.h>

static Balance_PID_FF_Gains_t gains;
static float integral_term = 0.0f;
#define INTEGRAL_MAX 5.0f   // 积分项限幅，防止积分饱和

static float rate_limited_steer_angle = 0.0f;

// 定义车把最大转向角速度 (rad/s)，可调参数， 例如： 2*PI rad/s = 360 deg/s
#define MAX_STEER_ANGULAR_VELOCITY_RAD_S (2.0f * M_PI)

static PID_Info_TypeDef pid_angle_outer; // 外环：角度环
static PID_Info_TypeDef pid_rate_inner;  // 内环：角速度环

static float angle_pid_param[PID_PARAMETER_NUM] = {
    5.0f,  // Kp
    0.0f,   // Ki
    0.0f,   // Kd
    0.0f,   // Alpha (关闭D项滤波)
    0.0035f,   // Deadband (无死区)
    0.0f,   // LimitIntegral (Ki为0，此值无影响，暂设为0)
    1.5f    // LimitOutput (输出最大角速度为 ±1.5 rad/s)
};

// 内环：角速度环参数
static float rate_pid_param[PID_PARAMETER_NUM] = {
    0.8f,   // Kp
    0.0f,   // Ki
    0.0f,  // Kd
    0.0f,   // Alpha (关闭D项滤波)
    0.0f,   // Deadband (无死区)
    0.0f,   // LimitIntegral (Ki为0，此值无影响，暂设为0)
    60.f * Rad_to_angle   // LimitOutput (输出最大转向角为 ±0.78 rad, 约±45度)
};

void Balance_Controller_Init(void)
{
    // 初始化外环 (角度环) - 主要靠P
    PID_Init(&pid_angle_outer, PID_POSITION, angle_pid_param);

    // 初始化内环 (角速度环) - P和D都非常重要
    PID_Init(&pid_rate_inner, PID_POSITION, rate_pid_param);
}

float Balance_Controller_Update(float current_roll_angle, float current_roll_rate)
{
    // 1. --- 执行外环计算 ---
    //    目标是保持角度为0
    //    输出是一个期望的角速度 (rad/s)
    float target_roll_rate = PID_Calculate(&pid_angle_outer, 0.0f, current_roll_angle);
    // printf("target_roll_rate: %f\r\n", target_roll_rate);
    // 2. --- 执行内环计算 ---
    //    目标是追踪外环给出的期望角速度
    //    输出是最终的转向角度 (rad)
    float target_steer_angle = PID_Calculate(&pid_rate_inner, target_roll_rate, current_roll_rate);
    // printf("target_steer_angle: %f\r\n", target_steer_angle);
    return target_steer_angle;
}

void Balance_Yu_Init(void)
{
    integral_term = 0.0f;
    // 设置一组初始的安全增益
    gains.Kp = 1.0f;
    gains.Ki = 0.f;
    gains.Kd = 0.f;
    gains.Kf = 8.5f; // 根据论文在1.3m/s速度下的实验值
}

void Balance_Yu_SetGains(float p, float i, float d, float f)
{
    gains.Kp = p;
    gains.Ki = i;
    gains.Kd = d;
    gains.Kf = f;
}

/**
 * @brief
 * @param desired_roll_rad      期望倾斜角 φd (rad)
 * @param actual_roll_rad       实际倾斜角 φ (rad)
 * @param actual_roll_rate_rad_s 实际倾斜角速度 φ_dot (rad/s)
 * @param dt                    时间步长 (s)
 * @return float                计算出的目标转向角 δd (rad)
 */
float Balance_Yu_Update(float desired_roll_rad, float actual_roll_rad, float actual_roll_rate_rad_s, float dt)
{
    float error = desired_roll_rad - actual_roll_rad;
    // printf("error: %f\r\n", error);
    float p_term = gains.Kp * error;

    integral_term += error * dt;
    if (integral_term > INTEGRAL_MAX) integral_term = INTEGRAL_MAX;
    if (integral_term < -INTEGRAL_MAX) integral_term = -INTEGRAL_MAX;
    float i_term = gains.Ki * integral_term;

    float d_term = gains.Kd * actual_roll_rate_rad_s;

    float ff_term = gains.Kf * desired_roll_rad;
    float raw_steer_angle_cmd = p_term + i_term - d_term + ff_term;

    // 对原始指令进行斜率限制 ---
    float max_change = MAX_STEER_ANGULAR_VELOCITY_RAD_S * dt;
    float error2 = raw_steer_angle_cmd - rate_limited_steer_angle;

    if (error2 > max_change) {
        rate_limited_steer_angle += max_change;
    } else if (error2 < -max_change) {
        rate_limited_steer_angle -= max_change;
    } else {
        rate_limited_steer_angle = raw_steer_angle_cmd;
    }
    // ------------------------------------

    // 添加输出限幅
    float max_steer_angle = 45.0f * (float)M_PI / 180.0f;
    if (rate_limited_steer_angle > max_steer_angle) rate_limited_steer_angle = max_steer_angle;
    if (rate_limited_steer_angle < -max_steer_angle) rate_limited_steer_angle = -max_steer_angle;

    // 返回平滑后的值
    return rate_limited_steer_angle;

    // printf("p_term: %f, i_term: %f, d_term: %f, ff_term: %f\r\n", p_term, i_term, d_term, ff_term);
    // float steer_angle_cmd = p_term + i_term - d_term + ff_term;
    // // printf("steer_angle_cmd: %f\r\n", steer_angle_cmd);
    //
    // // 添加输出限幅
    // float max_steer_angle = 45.0f * (float)M_PI / 180.0f; // 限制最大转向角为±45度
    // if (steer_angle_cmd > max_steer_angle) steer_angle_cmd = max_steer_angle;
    // if (steer_angle_cmd < -max_steer_angle) steer_angle_cmd = -max_steer_angle;
    //
    // return steer_angle_cmd;
}