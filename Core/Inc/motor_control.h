#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/*--- Ð?nh nghia ch? d? di chuy?n ---*/
#define normal_speed        800

typedef enum {
		turn_left,
		turn_right,
		go_ahead,
		stop,
    turn_left_imme,
    turn_right_imme,
    go_back,
} direction;

// + di thang, - di lui
enum {
		left_side, 	// IN1 +, IN2 -, TIMER CHANNEL 1 PA8
		right_side,	// IN3 +, IN4 -, TIMER CHANNEL 2 PA9
};

/*--- Khai báo các function di?u khi?n d?ng co ---*/
void MotorControl(int motor, int speed);
void TurnLeft();
void TurnRight();
void TurnLeftImme();
void TurnRightImme();
void GoBack();

#endif