//
// Created by Jelly on 2025/8/21.
//

#include "bsp_fdcan.h"

#include "DM_Motor.h"

/**
  * @brief FDCAN接收FIFO0回调函数
  * 当中断触发时，此函数被HAL库自动调用
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    // printf("FDCAN_RxFifo0Callback\r\n");
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        // 从接收FIFO0中获取消息
        if (hfdcan->Instance == FDCAN1) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                // 将收到的数据交给电机解析函数处理
                DJI_Motor_ParseFeedback(RxHeader.Identifier, RxData);
            }
        }
        // else if (hfdcan->Instance == FDCAN2) {
        //     if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        //         // 将收到的数据交给DM电机解析函数处理
        //         DM_Motor_ParseFeedback(RxData);
        //     }
        // }
    }
}



void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    //printf("FDCAN_RxFifo1Callback\r\n");

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK) {
            // DM电机反馈ID固定为0x000，具体信息在数据负载里
            DM_Motor_ParseFeedback(RxData);
        }
    }
}
