#ifndef _fsm_h
#define _fsm_h

//״̬����
typedef enum
{
	M_m,
	M_Param,
	I_m,
	D_m,
	x,
	y,
	z
	
}fsm_State;

//�¼�����
typedef enum
{
	no_matter,
	up,
	down,
	enter,
	esc
	
}fsm_Event;

int menu_num(fsm_State current_menu);


#endif