//
// Created by Jelly on 2025/8/21.
//

#include "bsp_sbus.h"

uint8_t at9s_rx[1]; //用于接收一个字节的数据
uint8_t SBUS_data[25]; //保存0x0f开头的所有数据
uint8_t SBUS_init = 0; //表示数据有效开始接收
uint8_t SBUS_end = 0; //表示数据结尾，停止接收
int SBUS_cnt = 0; //记录接收了多少的字节数
int at9s_i = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        at9s_i = 1; ///表示触发一次中断接收
        if (at9s_rx[0] == 0x0f && SBUS_init == 0) {
            //当识别到为0x0f开始时
            SBUS_data[0] = at9s_rx[0];
            SBUS_cnt++;
            SBUS_init = 1; //开启接收
        }

        if (SBUS_init == 1 && SBUS_cnt <= 23) {
            //保存中间数据
            SBUS_data[SBUS_cnt] = at9s_rx[0];
            SBUS_cnt++; //记录位数
        }

        if (SBUS_init == 1 && SBUS_end == 0 && at9s_rx[0] == 0x00 && SBUS_cnt == 24) {
            // 结束
            SBUS_data[SBUS_cnt] = at9s_rx[0];
            SBUS_end = 1;
        }
        HAL_UART_Receive_IT(&huart5, at9s_rx, 1); //发生接收中断后，中断使能位会清除，故此处再调用此函数使能接收中断
    }
}
