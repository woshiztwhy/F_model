#include "fsm.h"

int menu_num(fsm_State current_menu)
{
	switch(current_menu)
	{
		case M_m:
			return 3;
		case M_Param:
			return 2;
		default:
			break;
	}
	return 0;
}
