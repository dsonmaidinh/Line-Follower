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
	
	MotorControl(right_side, 999);
	MotorControl(left_side, 700);
	GPIOB->ODR |= (1<<3) | (1<<6); // 4,6 di thang
	
}
void TurnRight(){
	GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
	
	MotorControl(left_side, 999 );
	MotorControl(right_side, 700);
	GPIOB->ODR |= (1<<4) | (1<<5);
}
void GoAhead(){
	GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
	
	MotorControl(left_side, normal_speed);
	MotorControl(right_side, normal_speed);
	GPIOB->ODR |= (1<<4) | (1<<6);
}
void Stop(){
	GPIOB->ODR &= ~((1 << 3) | (1 << 4) | (1 << 5) |(1 << 6));
	MotorControl(left_side, 0);
	MotorControl(right_side,0);
}
