//
// Created by Jelly on 2025/8/21.
//

#include "DM_Motor.h"

volatile float dm_target_position_rad = -0.f;
volatile DM_Motor_Feedback_t dm_motor_feedback[1];

void DM_Motor_init(void) {
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID; //标准ID
    fdcan_filter.FilterIndex = 0; //滤波器索引
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; //过滤器0关联到FIFO0
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);//根据过滤器结构体中指定的参数配置FDCAN接收过滤器
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); //使能中断，FIFO1新消息中断
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2); //使能FDCAN发送延时补偿
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2, 14, 14); //FDCAN发送延时补偿时间设置
    HAL_FDCAN_Start(&hfdcan2); //FDCAN开始工作
    HAL_FDCAN_ActivateNotification(&hfdcan2,
                                       0 | FDCAN_IT_RX_FIFO1_WATERMARK | FDCAN_IT_RX_FIFO1_WATERMARK
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       0x00000F00);

}

void DM_Motor_Enable(uint16_t motor_id, uint16_t mode_id) {
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    // printf("id: %x\r\n", id);

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    DM_FDCAN_Send_Data(&hfdcan2, id, data, 8);
    // printf("Motor %d enabled in mode %d\r\n", motor_id, mode_id);
}

void DM_Motor_Disable(uint16_t motor_id, uint16_t mode_id) {
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    DM_FDCAN_Send_Data(&hfdcan2, id, data, 8);
}

uint8_t DM_FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len) {
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier = id; // 设置CAN ID
    pTxHeader.IdType = FDCAN_STANDARD_ID; // 设置为标准ID
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME; // 设置为数据帧
    // 设置发送的数据长度
    if (len <= 8)
        pTxHeader.DataLength = len;
    else if (len == 12)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    else if (len == 16)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    else if (len == 20)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    else if (len == 24)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    else if (len == 32)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    else if (len == 48)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    else if (len == 64)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 传输节点 error active
    pTxHeader.BitRateSwitch = FDCAN_BRS_ON; // FDCAN 帧发送/接收带波特率可变
    pTxHeader.FDFormat = FDCAN_FD_CAN; // 设置为FDCAN帧格式，兼容CAN
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不存储 Tx events 事件
    pTxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) {
        // printf("FDCAN Send Error\r\n");
        return 1; //发送
    }
    // printf("FDCAN Send Data, ");
    // printf("data is :");
    // for (uint32_t i = 0; i < len; i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\r\n");
    return 0;
}

void DM_Motor_Pos_Ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel) {
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + POS_MODE;
    pbuf = (uint8_t *) &pos;
    vbuf = (uint8_t *) &vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    DM_FDCAN_Send_Data(hcan, id, data, 8);
}

void DM_Motor_ParseFeedback(uint8_t *rx_data) {
    uint8_t motor_id = rx_data[0] & 0x0F;
    // printf("motor id: %x\r\n", motor_id);
    if (motor_id > 0) // 假设ID>0
    {
        dm_motor_feedback[motor_id - 1].id = motor_id;
        dm_motor_feedback[motor_id - 1].state = (rx_data[0]) >> 4;

        uint16_t p_int = (rx_data[1] << 8) | rx_data[2];
        uint16_t v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        uint16_t t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5]; // 注意手册版本，这里有的是6字节有的是8字节反馈

        dm_motor_feedback[motor_id - 1].position = uint_to_float(p_int, P_MIN, P_MAX, 16);
        dm_motor_feedback[motor_id - 1].velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
        dm_motor_feedback[motor_id - 1].torque = uint_to_float(t_int, T_MIN, T_MAX, 12);
    }
    //todo: DM电机无回传数据，待解决
}

void Set_Bicycle_Angle(float angle_rad) {
    dm_target_position_rad = angle_rad;
}

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return (uint16_t) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

// 辅助函数：将无符号整数根据范围映射回浮点数
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

void write_motor_data(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

    uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
    DM_FDCAN_Send_Data(&hfdcan2, 0x7FF, data, 8);
}

void save_motor_data(uint16_t id, uint8_t rid)
{
    uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

    uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
    DM_FDCAN_Send_Data(&hfdcan2, 0x7FF, data, 4);
}