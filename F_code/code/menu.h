#ifndef _menu_h
#define _menu_h

#include "zf_common_typedef.h"
#include "zf_device_ips200.h"
#include "PID.h"
//状态定义
typedef enum
{
	M_m,
	M_Param,
	I_mode,
	D_m,
	M_duty,
	M_set_speed,
	Dir_PID_State,
	S_PID_State,
	Grey_Im,
	Binary_Im,
	//****PID****
	Speed_p,
	Speed_i,
	Speed_d,
	Dir_p,
	Dir_i,
	Dir_d,
	//****PID****
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
void motor_set_speed_menu_init(int set_speed);
void motor_speed_pid_init(PID *pid);
void departure_mode_init(void);
void motor_dir_pid_init(float pid[]);


#endif