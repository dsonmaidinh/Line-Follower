#include "motor_control.h"

void MotorControl(int motor, int speed){
	if (speed > 999) speed = 999;
	if (speed < 0) speed = 0;
	if(motor == left_side){
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, speed);	// 2 banh ben trai PA8
	}
	else{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, speed);	// 2 banh ben phai PA9
	}
}
void TurnLeft(){
	GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
	GPIOB->ODR |= (1<<4) | (1<<6);
	MotorControl(right_side, 800);
	for(int16_t i = 800; i>400; i-=20){
		MotorControl(left_side, i);
		HAL_Delay(1);
	}
}
void TurnRight(){
	GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
	GPIOB->ODR |= (1<<4) | (1<<6);
	MotorControl(left_side, 800);
	MotorControl(right_side, 400);
}
