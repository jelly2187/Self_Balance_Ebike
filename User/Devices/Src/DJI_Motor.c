//
// Created by Jelly on 2025/8/21.
//

#include "DJI_Motor.h"
#include <math.h>
#define FRICTION_WHEEL_DIAMETER_METERS (0.063f)

PID_Controller_t dji_motor_speed_pid;
volatile float dji_target_speed_rpm = -000.0f;
// 定义电机反馈数据数组
volatile DJI_Motor_Feedback_t motor_feedback[1];


// FDCAN句柄，在main.c中定义
extern FDCAN_HandleTypeDef hfdcan1;

/**
  * @brief 初始化电机相关配置，主要是FDCAN过滤器
  */
void DJI_Motor_Init(void)
{
    FDCAN_FilterTypeDef FDCAN1_sFilterConfig;

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    // 激活接收FIFO0新消息通知
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    // 配置CAN过滤器，只接收电机ID为1的反馈报文(ID: 0x201)
    FDCAN1_sFilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_sFilterConfig.FilterIndex = 0;
    FDCAN1_sFilterConfig.FilterType = FDCAN_FILTER_MASK; // 掩码模式
    FDCAN1_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_sFilterConfig.FilterID1 = 0x000; // 要匹配的ID
    FDCAN1_sFilterConfig.FilterID2 = 0x000; // 掩码，0x7FF表示ID的每一位都必须精确匹配
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_sFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
    // printf("DJI Motor Initialized\r\n");
}

/**
  * @brief 向C620电调发送控制指令(电流值)
  * @param motor1_current: ID为1的电机的目标电流值 (-16384 to 16384)
  */
void DJI_Motor_SendCommand(int16_t motor1_current)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];

    TxHeader.Identifier = 0x200; // 使用0x200 ID控制1-4号电机
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 填充数据负载
    // 只控制1号电机，其他电机指令为0
    TxData[0] = (motor1_current >> 8) & 0xFF; // motor1 高8位
    TxData[1] = motor1_current & 0xFF;        // motor1 低8位
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;

    // 将消息添加到发送FIFO
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
    {
        // 发送失败处理
    }
}

/**
  * @brief 解析从电机接收到的反馈数据
  * @param can_id: 接收到的CAN ID
  * @param data: 指向8字节数据的指针
  */
void DJI_Motor_ParseFeedback(uint32_t can_id, uint8_t *data)
{
    // 检查是否是ID为1的电机反馈 (ID 0x201)
    if (can_id == 0x201)
    {
        motor_feedback[0].angle         = (uint16_t)(data[0] << 8 | data[1]);
        motor_feedback[0].speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
        motor_feedback[0].torque_current = (int16_t)(data[4] << 8 | data[5]);
        motor_feedback[0].temperature   = data[6];
        // printf("%d %d %d %d\r\n",
        //        motor_feedback[0].angle,
        //        motor_feedback[0].speed_rpm,
        //        motor_feedback[0].torque_current,
        //        motor_feedback[0].temperature);
    }
}

void Set_Bicycle_Speed(float bicycle_speed_mps)
{
    dji_target_speed_rpm = (bicycle_speed_mps * 60.0f) / (FRICTION_WHEEL_DIAMETER_METERS * 3.1415926f);
}


float Get_Bicycle_Speed(void) {
    return (motor_feedback[0].speed_rpm * FRICTION_WHEEL_DIAMETER_METERS * 3.1415926f) / 60.0f;
}