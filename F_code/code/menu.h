#ifndef _menu_h
#define _menu_h

//状态定义
typedef enum
{
	M_m,
	M_Param,
	I_m,
	D_m,
	M_duty,
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

int menu_num(fsm_State current_menu);

void main_menu_init(void);
void motor_param_menu_init(void);
void motor_duty_menu_init(int duty);


#endif