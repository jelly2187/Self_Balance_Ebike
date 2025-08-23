//
// Created by Jelly on 2025/8/21.
//

#include "at9s_pro.h"

#include "bsp_mcu.h"

uint16_t SBUS_thoroughfare[16] = {0};

void Remote_Control_Update(void) {
    if (at9s_i == 1) {
        at9s_i = 0;
        //HAL_UART_Transmit(&huart2,&data, 3, 0xff);
        if (SBUS_end == 1) {
            //HAL_UART_Transmit(&huart2, SBUS_data, 25, 0xff);
            SBUS_thoroughfare_analysis(0); //HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 5, 0xff);
            //printf("%d\r\n",SBUS_data[0]);
            SBUS_end = 0;
            SBUS_init = 0;
            SBUS_cnt = 0;
            // HAL_Delay(1000);
        }
    }
}

void Sbus_Data_Count(uint8_t *buf) //将8*22=176字节重新放入到SBUS_thoroughfare通道数据中
{
    SBUS_thoroughfare[0] = ((int16_t) buf[2] >> 0 | ((int16_t) buf[3] << 8)) & 0x07FF;
    SBUS_thoroughfare[1] = ((int16_t) buf[3] >> 3 | ((int16_t) buf[4] << 5)) & 0x07FF;
    SBUS_thoroughfare[2] = ((int16_t) buf[4] >> 6 | ((int16_t) buf[5] << 2) | (int16_t) buf[6] << 10) & 0x07FF;
    SBUS_thoroughfare[3] = ((int16_t) buf[6] >> 1 | ((int16_t) buf[7] << 7)) & 0x07FF;
    SBUS_thoroughfare[4] = ((int16_t) buf[7] >> 4 | ((int16_t) buf[8] << 4)) & 0x07FF;
    SBUS_thoroughfare[5] = ((int16_t) buf[8] >> 7 | ((int16_t) buf[9] << 1) | (int16_t) buf[10] << 9) & 0x07FF;
    SBUS_thoroughfare[6] = ((int16_t) buf[10] >> 2 | ((int16_t) buf[11] << 6)) & 0x07FF;
    SBUS_thoroughfare[7] = ((int16_t) buf[11] >> 5 | ((int16_t) buf[12] << 3)) & 0x07FF;

    SBUS_thoroughfare[8] = ((int16_t) buf[13] << 0 | ((int16_t) buf[14] << 8)) & 0x07FF;
    SBUS_thoroughfare[9] = ((int16_t) buf[14] >> 3 | ((int16_t) buf[15] << 5)) & 0x07FF;
    SBUS_thoroughfare[10] = ((int16_t) buf[15] >> 6 | ((int16_t) buf[16] << 2) | (int16_t) buf[17] << 10) & 0x07FF;
    SBUS_thoroughfare[11] = ((int16_t) buf[17] >> 1 | ((int16_t) buf[18] << 7)) & 0x07FF;
    SBUS_thoroughfare[12] = ((int16_t) buf[18] >> 4 | ((int16_t) buf[19] << 4)) & 0x07FF;
    SBUS_thoroughfare[13] = ((int16_t) buf[19] >> 7 | ((int16_t) buf[20] << 1) | (int16_t) buf[21] << 9) & 0x07FF;
    SBUS_thoroughfare[14] = ((int16_t) buf[21] >> 2 | ((int16_t) buf[22] << 6)) & 0x07FF;
    SBUS_thoroughfare[15] = ((int16_t) buf[22] >> 5 | ((int16_t) buf[23] << 3)) & 0x07FF;
}

void SBUS_thoroughfare_analysis(int n) {
    Sbus_Data_Count(SBUS_data); //解析数据
    if (n == 1) {
        printf("======================================\r\n");
        printf("yes: head=0x0F, flag=0x00, end=0x00\r\n\r\n");
        printf("head: %d\r\n", SBUS_data[0]);
        printf("  %4d, %4d, %4d, %4d\r\n", SBUS_thoroughfare[0], SBUS_thoroughfare[1], SBUS_thoroughfare[2],
               SBUS_thoroughfare[3]);
        printf("  %4d, %4d, %4d, %4d\r\n", SBUS_thoroughfare[4], SBUS_thoroughfare[5], SBUS_thoroughfare[6],
               SBUS_thoroughfare[7]);
        printf("  %4d, %4d, %4d, %4d\r\n", SBUS_thoroughfare[8], SBUS_thoroughfare[9], SBUS_thoroughfare[10],
               SBUS_thoroughfare[11]);
        printf("  %4d, %4d, %4d, %4d\r\n", SBUS_thoroughfare[12], SBUS_thoroughfare[13], SBUS_thoroughfare[14],
               SBUS_thoroughfare[15]);
        printf("flag: %4d\r\n", SBUS_data[23]);
        printf("end : %4d\r\n", SBUS_data[24]);
        printf("======================================\r\n\r\n");
    }
    printf("%d\r\n", SBUS_thoroughfare[3]);
    //printf("%d\r\n",SBUS_thoroughfare[5]);
    //printf("  %4d, %4d, %4d, %4d\r\n", SBUS_thoroughfare[0], SBUS_thoroughfare[1], SBUS_thoroughfare[2], SBUS_thoroughfare[3]);
}

// Reference: https://blog.csdn.net/yandadzf/article/details/135743995
