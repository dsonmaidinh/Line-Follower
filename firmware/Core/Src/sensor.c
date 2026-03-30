#include "sensor.h"
#include "motor_control.h"

void line_follow(){
		if ( S2 == Thay_Line && S4 == Khong_Thay_Line){
			TurnRight();
			test = 3;
	 }
		else if( S2 == Khong_Thay_Line && S4 == Thay_Line){
			TurnLeft();
			test = 2;
	 }
	 else if ( S2 == Khong_Thay_Line && S4 == Khong_Thay_Line){
			GoAhead();
			test = 1;
	}
	else {
			Stop();
			test = 0;
	}
}





