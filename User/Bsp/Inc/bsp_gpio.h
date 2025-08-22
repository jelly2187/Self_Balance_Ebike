//
// Created by Jelly on 2025/8/22.
//

#ifndef BALANCE_EBIKE_BSP_GPIO_H
#define BALANCE_EBIKE_BSP_GPIO_H

#ifdef __cplusplus
extern "C" {

#endif

/* Externs ------------------------------------------------------------------*/
extern void BSP_GPIO_Init(void);

extern void BMI088_ACCEL_NS_L(void);

extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);

extern void BMI088_GYRO_NS_H(void);

#endif //BALANCE_EBIKE_BSP_GPIO_H
