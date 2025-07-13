#include "menu.h"


uint8 menu_num(fsm_State current_menu)//获取选项数
{
	switch(current_menu)
	{
		case M_m:
			return 3;
		case M_Param:
			return 3;
		case I_mode:
			return 2;
		case S_PID_State:
			return 3;
		case Dir_PID_State:
			return 3;
		default:
			break;
	}
	return 0;
}

//******************菜单初始化****************************
void main_menu_init(void)
{
	ips200_clear();
	ips200_show_string(0,0,"Main menu");
	ips200_show_string(24,16,"Motor Paramter");
	ips200_show_string(24,32,"Image mode");
	ips200_show_string(24,48,"Departure mode");
}

void motor_param_menu_init(void)
{
	ips200_clear();
	ips200_show_string(0,0,"Motor param");
	ips200_show_string(24,16,"Direction PID");
	ips200_show_string(24,32,"Speed PID");
	ips200_show_string(24,48,"Motor set_speed");
}

void motor_set_speed_menu_init(int set_speed)
{
	ips200_clear();
	ips200_show_string(0,0,"Set_speed:");
	ips200_show_int(96,0,set_speed,4);
}

void image_mode_menu_init(void)
{
	ips200_clear();
	ips200_show_string(0,0,"Image mode");
	ips200_show_string(24,16,"Binary image");
	ips200_show_string(24,32,"Grey image");
}

void motor_speed_pid_init(PID *pid)
{
	ips200_clear();
	ips200_show_string(0,0,"Speed PID:");
	ips200_show_string(32,16,"P:");
	ips200_show_float(64,16,pid->kp,1,4);
	ips200_show_string(32,32,"I:");
	ips200_show_float(64,32,pid->ki,1,4);
	ips200_show_string(32,48,"D:");
	ips200_show_float(64,48,pid->kd,1,4);
}

void motor_dir_pid_init(PID *pid)
{
	ips200_clear();
	ips200_show_string(0,0,"Dir PID:");
	ips200_show_string(32,16,"P:");
	ips200_show_float(64,16,pid->kp,1,5);
	ips200_show_string(32,32,"I:");
	ips200_show_float(64,32,pid->ki,1,5);
	ips200_show_string(32,48,"D:");
	ips200_show_float(64,48,pid->kd,1,5);
}

void departure_mode_init(void)
{
	ips200_clear();
	ips200_show_string(0,16,"avg:");
	ips200_show_string(0,48,"s_out:");
}
//******************菜单初始化****************************