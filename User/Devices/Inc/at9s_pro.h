//
// Created by Jelly on 2025/8/21.
//

#ifndef BALANCE_EBIKE_AT9S_PRO_H
#define BALANCE_EBIKE_AT9S_PRO_H

#include "main.h"
#include "bsp_sbus.h"

extern uint16_t SBUS_thoroughfare[16];

void Remote_Control_Update(void);

void Sbus_Data_Count(uint8_t *buf);

void SBUS_thoroughfare_analysis(int n);
#endif //BALANCE_EBIKE_AT9S_PRO_H
