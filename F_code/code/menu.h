#ifndef _menu_h
#define _menu_h

#include "zf_common_typedef.h"
//状态定义
typedef enum
{
	M_m,
	M_Param,
	I_mode,
	D_m,
	M_duty,
	Grey_Im,
	Binary_Im,
	x,
	y,
	z
	
}fsm_State;

//事件定义
typedef enum
{
	no_matter,
	up,
	down,
	enter,
	esc
	
}fsm_Event;

uint8 menu_num(fsm_State current_menu);

void main_menu_init(void);
void motor_param_menu_init(void);
void motor_duty_menu_init(int duty);
void image_mode_menu_init(void);


#endif