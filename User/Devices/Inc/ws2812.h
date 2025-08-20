//
// Created by Jelly on 25-6-26.
//

#ifndef WS2812_H
#define WS2812_H
#include "main.h"


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
#endif //WS2812_H
