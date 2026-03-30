#ifndef sensor_H
#define sensor_H

#include "stm32f1xx_hal.h"

#define S1 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)
#define S2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)
#define S3 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)
#define S4 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)
#define Khong_Thay_Line 			GPIO_PIN_RESET
#define Thay_Line 						GPIO_PIN_SET

//S1: B10, S2: B1, S3: B11 S4: B0
extern uint8_t test;

void line_follow();
 
#endif
