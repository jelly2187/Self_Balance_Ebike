//
// Created by Jelly on 2025/8/22.
//

#include "INS.h"
#include "BMI088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"
#include <math.h>


#define IMU_CALIBRATION_ENABLE   0U     // 1: 执行校准流程, 0: 使用硬编码的校准值
uint16_t GYRO_CAL_SAMPLE_COUNT = 30000; // 在1kHz下采样2000次，耗时2秒
float TARGET_TEMP = 40.0f; // 目标预热温度

static volatile INS_State_e ins_state = INS_STATE_UNINIT; // 系统状态
static int32_t gyro_offset_sum[3] = {0, 0, 0};
uint16_t cal_sample_count = 0;
static uint32_t stable_temp_counter = 0;

/**
  * @brief the structure that contains the information for the INS.
  */
INS_Info_Typedef INS_Info;

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float INS_LPF2p_Alpha[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];

/**
  * @brief the Initialize data of state transition matrix.
  */
static float QuaternionEKF_A_Data[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

/**
  * @brief the Initialize data of posteriori covariance matrix.
  */
static float QuaternionEKF_P_Data[36] = {
    100000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 100000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 100000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 100000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 100, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 100
};

/**
  * @brief the Initialize data of Temperature Control PID.
  */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM] = {900, 20, 0, 0, 0, 0, 2000};

/**
  * @brief the structure that contains the Information of Temperature Control PID.
  */
PID_Info_TypeDef TempCtrl_PID;

static void BMI088_Temp_Control(float temp);

void INS_Init(void) {
    BMI088_Init();
    /* Initializes the Second order lowpass filter  */
    LowPassFilter2p_Init(&INS_AccelPF2p[0], INS_LPF2p_Alpha);
    LowPassFilter2p_Init(&INS_AccelPF2p[1], INS_LPF2p_Alpha);
    LowPassFilter2p_Init(&INS_AccelPF2p[2], INS_LPF2p_Alpha);

    /* Initializes the Temperature Control PID  */
    PID_Init(&TempCtrl_PID, PID_POSITION, TemCtrl_PID_Param);

    /* Initializes the Quaternion EKF */
    QuaternionEKF_Init(&Quaternion_Info, 10.f, 0.001f, 1000000.f, QuaternionEKF_A_Data, QuaternionEKF_P_Data);

    // 初始化零偏值为0
    BMI088_Info.Offsets_Gyro_X = 0.0f;
    BMI088_Info.Offsets_Gyro_Y = 0.0f;
    BMI088_Info.Offsets_Gyro_Z = 0.0f;
    BMI088_Info.Offsets_Init = false;

#if IMU_CALIBRATION_ENABLE
    // 如果使能校准，则进入预热状态
    ins_state = INS_STATE_PREHEATING;
#else
    // 如果不使能校准，则直接使用硬编码的校准值
    BMI088_Info.Offsets_Gyro_X = 0.002006; // 在这里填入您之前校准好的值
    BMI088_Info.Offsets_Gyro_Y = 0.000865; // 在这里填入您之前校准好的值
    BMI088_Info.Offsets_Gyro_Z = 0.003156; // 在这里填入您之前校准好的值
    BMI088_Info.Offsets_Init = true;
    ins_state = INS_STATE_RUNNING; // 直接进入运行状态
#endif
}

/**
 * @brief 获取当前系统状态
 */
INS_State_e INS_Get_State(void) {
    return ins_state;
}

//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature
  * @param  temp  measure of the BMI088 temperature
  * @retval none
  */
static void BMI088_Temp_Control(float Temp) {
    PID_Calculate(&TempCtrl_PID, 40.f, Temp);

    VAL_LIMIT(TempCtrl_PID.Output, -TempCtrl_PID.Param.LimitOutput, TempCtrl_PID.Param.LimitOutput);

    Heat_Power_Control((uint16_t) (TempCtrl_PID.Output));
}


void INS_Update(void) {
    // 用于实现5ms执行一次温度控制的计数器
    // static uint32_t temp_ctrl_counter = 0;

    /* 更新BMI088测量值 */
    BMI088_Info_Update(&BMI088_Info);

    // 根据当前状态执行不同逻辑
    switch (ins_state) {
        case INS_STATE_UNINIT:
            // 不执行任何操作
            break;
        case INS_STATE_PREHEATING:
            // 持续进行温度控制
            BMI088_Temp_Control(BMI088_Info.Temperature);

            // 检查温度是否达到目标
            if (fabsf(BMI088_Info.Temperature - TARGET_TEMP) < TEMP_STABILIZATION_TOLERANCE)
            {
                // 如果在范围内，稳定计时器自增
                if (stable_temp_counter < TEMP_STABILIZATION_DURATION_MS)
                {
                    stable_temp_counter++;
                }
            }
            else
            {
                // 如果温度超出了范围，立刻将计时器清零
                stable_temp_counter = 0;
            }

            // 检查稳定计时器是否已达到要求的时间
            if (stable_temp_counter >= TEMP_STABILIZATION_DURATION_MS)
            {
                // 温度已稳定达标，自动切换到陀螺仪校准状态

                // 清空累加器和计数器，准备校准 (这部分逻辑不变)
                for(int i=0; i<3; i++) { gyro_offset_sum[i] = 0; }
                cal_sample_count = 0;

                // 切换状态
                ins_state = INS_STATE_GYRO_CALIBRATING;
            }
            break;
        case INS_STATE_GYRO_CALIBRATING:
            // 在此状态下，设备必须保持静止
            // BMI088_Offset_Update(&BMI088_Info);
            // uint8_t buf[8] = {0,};
            // 累加陀螺仪原始数据
            gyro_offset_sum[0] += BMI088_Info.MPU_Info.Gyro_X;
            gyro_offset_sum[1] += BMI088_Info.MPU_Info.Gyro_Y;
            gyro_offset_sum[2] += BMI088_Info.MPU_Info.Gyro_Z;
            cal_sample_count++;
            // Delay_ms(1);

            // 检查是否已采集足够样本
            if (cal_sample_count >= GYRO_CAL_SAMPLE_COUNT) {
                // 计算平均零偏值
                BMI088_Info.Offsets_Gyro_X = (float) gyro_offset_sum[0] / cal_sample_count * BMI088_GYRO_SEN;
                BMI088_Info.Offsets_Gyro_Y = (float) gyro_offset_sum[1] / cal_sample_count * BMI088_GYRO_SEN;
                BMI088_Info.Offsets_Gyro_Z = (float) gyro_offset_sum[2] / cal_sample_count * BMI088_GYRO_SEN;

                // 标记校准完成
                BMI088_Info.Offsets_Init = true;
                ins_state = INS_STATE_RUNNING;
            }
            break;
        case INS_STATE_RUNNING:
            // 校准完成，进入正常姿态解算和控制流程

            // 正常运行时，也需要持续进行温度控制 (每5ms一次)
            static uint32_t temp_ctrl_counter = 0;
            if (++temp_ctrl_counter >= 5) {
                temp_ctrl_counter = 0;
                BMI088_Temp_Control(BMI088_Info.Temperature);
            }
            /* 加速度计二阶低通滤波 */
            INS_Info.Accel[0] = LowPassFilter2p_Update(&INS_AccelPF2p[0], BMI088_Info.Accel[0]);
            INS_Info.Accel[1] = LowPassFilter2p_Update(&INS_AccelPF2p[1], BMI088_Info.Accel[1]);
            INS_Info.Accel[2] = LowPassFilter2p_Update(&INS_AccelPF2p[2], BMI088_Info.Accel[2]);

            /* 更新陀螺仪测量值 (单位: rad) */
            INS_Info.Gyro[0] = BMI088_Info.Gyro[0];
            INS_Info.Gyro[1] = BMI088_Info.Gyro[1];
            INS_Info.Gyro[2] = BMI088_Info.Gyro[2];

            /* 更新四元数EKF */
            // 注意：EKF的时间步长0.001f与我们的定时器中断周期1ms是严格对应的
            QuaternionEKF_Update(&Quaternion_Info, INS_Info.Gyro, INS_Info.Accel, 0.001f);

            /* 更新欧拉角 (单位: rad) */
            memcpy(INS_Info.Angle, Quaternion_Info.EulerAngle, sizeof(INS_Info.Angle));

            /* 更新欧拉角 (单位: 度) */
            INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH] * 57.295779513f;
            INS_Info.Yaw_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW] * 57.295779513f;
            INS_Info.Roll_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL] * 57.295779513f;

            /* 更新Yaw轴总角度 */
            if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f) {
                INS_Info.YawRoundCount++;
            } else if (INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f) {
                INS_Info.YawRoundCount--;
            }
            INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
            INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount * 360.f;

            /* 更新角速度 (单位: 度/秒) */
            INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH] * RadiansToDegrees;
            INS_Info.Yaw_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_YAW] * RadiansToDegrees;
            break;
    }


    // INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;

    /* 每5ms执行一次温度控制 */
    // if (++temp_ctrl_counter >= 5) {
    //     temp_ctrl_counter = 0;
    //     BMI088_Temp_Control(BMI088_Info.Temperature);
    // }
}

void INS_Get_Preheating_Progress(uint32_t *stable_time_ms)
{
    *stable_time_ms = stable_temp_counter;
}
