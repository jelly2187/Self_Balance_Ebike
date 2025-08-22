//
// Created by Jelly on 2025/8/22.
//

#include "INS.h"
#include "bmi088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"

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
    /* Initializes the Second order lowpass filter  */
    LowPassFilter2p_Init(&INS_AccelPF2p[0], INS_LPF2p_Alpha);
    LowPassFilter2p_Init(&INS_AccelPF2p[1], INS_LPF2p_Alpha);
    LowPassFilter2p_Init(&INS_AccelPF2p[2], INS_LPF2p_Alpha);

    /* Initializes the Temperature Control PID  */
    PID_Init(&TempCtrl_PID, PID_POSITION, TemCtrl_PID_Param);

    /* Initializes the Quaternion EKF */
    QuaternionEKF_Init(&Quaternion_Info, 10.f, 0.001f, 1000000.f, QuaternionEKF_A_Data, QuaternionEKF_P_Data);
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
    static uint32_t temp_ctrl_counter = 0;

    /* 更新BMI088测量值 */
    BMI088_Info_Update(&BMI088_Info);

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
    // INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;

    /* 每5ms执行一次温度控制 */
    if (++temp_ctrl_counter >= 5) {
        temp_ctrl_counter = 0;
        BMI088_Temp_Control(BMI088_Info.Temperature);
    }
}
