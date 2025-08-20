//
// Created by Jelly on 2025/8/21.
//

#ifndef BALANCE_EBIKE_BSP_SBUS_H
#define BALANCE_EBIKE_BSP_SBUS_H

#include "main.h"
#include "usart.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_uart.h"

extern uint8_t at9s_rx[1]; //用于接收一个字节的数据
extern uint8_t SBUS_data[25]; //保存0x0f开头的所有数据
extern uint8_t SBUS_init; //表示数据有效开始接收
extern uint8_t SBUS_end; //表示数据结尾，停止接收
extern int SBUS_cnt; //记录接收了多少的字节数
extern int at9s_i;
#endif //BALANCE_EBIKE_BSP_SBUS_H
