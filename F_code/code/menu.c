#include "menu.h"
#include "zf_device_ips200.h"

uint8 menu_num(fsm_State current_menu)//获取选项数
{
	switch(current_menu)
	{
		case M_m:
			return 3;
		case M_Param:
			return 2;
		case I_mode:
			return 2;
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
	ips200_show_string(24,16,"Motor duty");
	ips200_show_string(24,32,"Back to main");
}

void motor_duty_menu_init(int duty)
{
	ips200_clear();
	ips200_show_string(0,0,"Motor duty:");
	ips200_show_int(96,0,duty,4);
}

void image_mode_menu_init(void)
{
	ips200_clear();
	ips200_show_string(0,0,"Image mode");
	ips200_show_string(24,16,"Binary image");
	ips200_show_string(24,32,"Grey image");
}

//******************菜单初始化****************************