//
// Created by Jelly on 2025/8/22.
//

#ifndef BALANCE_EBIKE_BSP_PWM_H
#define BALANCE_EBIKE_BSP_PWM_H
#include "stdint.h"
#include "tim.h"

extern void BSP_PWM_Init(void);

/**
  * @brief  Set the BMI088 Heat_Power TIM Capture Compare Register value.
  */
extern void Heat_Power_Control(uint16_t Compare);

/**
  * @brief  Set the TIM Capture Compare Register value.
  */
extern void USER_Tim_SetCompare(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t Compare);
#endif //BALANCE_EBIKE_BSP_PWM_H
