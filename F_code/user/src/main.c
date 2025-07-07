/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// *************************** 例程硬件连接说明 ***************************
//      将核心板插在主板上 确保插到底核心板与主板插座间没有缝隙即可
//      将核心板插在主板上 确保插到底核心板与主板插座间没有缝隙即可
//      将核心板插在主板上 确保插到底核心板与主板插座间没有缝隙即可
// 
//      主板按键            单片机管脚
//      S1                  查看 zf_device_key.h 文件中 KEY1_PIN 宏定义的引脚 默认 E2
//      S2                  查看 zf_device_key.h 文件中 KEY2_PIN 宏定义的引脚 默认 E3
//      S3                  查看 zf_device_key.h 文件中 KEY3_PIN 宏定义的引脚 默认 E4
//      S4                  查看 zf_device_key.h 文件中 KEY4_PIN 宏定义的引脚 默认 E5
// 
//      主板拨码开关        单片机管脚
//      S5-1                D14
//      S5-2                D15
// 
//      主板蜂鸣器          单片机管脚
//      BEEP                D12


// *************************** 例程测试说明 ***************************
// 1.核心板插在主板上 主板使用电池供电 下载本例程
// 
// 2.复位核心板 两个LED会先闪烁两次 蜂鸣器也会响两次
// 
// 3.短按一下 S1-S4 中任意按键 BEEP 会响一下
// 
// 4.长按 S1-S4 中任意按键 BEEP 会一直响
// 
// 5.拨动 SWITCH1/SWITCH2 拨码开关 LED1/LED2 会切换闪烁或者熄灭状态
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************

#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define DIR_L               (A0 )
#define PWM_L               (TIM5_PWM_CH2_A1)

#define DIR_R               (A2 )
#define PWM_R               (TIM5_PWM_CH4_A3)

#define ENCODER_1                   (TIM3_ENCODER)
#define ENCODER_1_A                 (TIM3_ENCODER_CH1_B4)
#define ENCODER_1_B                 (TIM3_ENCODER_CH2_B5)

#define ENCODER_2                   (TIM4_ENCODER)
#define ENCODER_2_A                 (TIM4_ENCODER_CH1_B6)
#define ENCODER_2_B                 (TIM4_ENCODER_CH2_B7)

#define PIT                         (TIM6_PIT )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

int Left_duty = 0;
int Right_duty = 0;
bool dir = true;
int Threshold = 0;
int16 Speed_Left_Real=0;
int16 Speed_Right_Real=0;
int16 Speed_Real=0;
int16 Speed_Set=100;

PID Direction_PID={0};
PID Speed_PID={0};

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	
	system_delay_ms(300); 
                                                               
   // 此处编写用户代码 例如外设初始化代码等
	ips200_init(IPS200_TYPE_SPI);
	
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
	
	key_init(10); 
	
	encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器模块与引脚 
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 
    pit_ms_init(PIT, 100);
	
	PID_Init(&Direction_PID,0,0,0,8,10);
	PID_Init(&Speed_PID,0,0,0,8,10);
	
	//**********************总钻风初始化***********************
	ips200_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
        if(mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            break;
        }
        system_delay_ms(500);                                                   // 短延时快速闪灯表示异常
    }
    ips200_show_string(0, 16, "init success.");
	system_delay_ms(500);
	//**********************总钻风初始化************************
	
	main_menu_init();
	ips200_show_string(0,16,"->");
	
	uint16 loop = 0;
	
	fsm_State current_state=M_m;
	
	uint8 current_p=0;
	
    while(1)
	{
		//************************图像处理****************************
		if(mt9v03x_finish_flag)                                                    //先处理图像，再清除标志位
		{
			Threshold=My_Adapt_Threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);   //大津算阈值
			Image_Binarization(Threshold);                                         //图像二值化
			Longest_White_Column();                                                //最长白线法寻边线
			Cross_Detect();                                                        //十字检测补线
			mt9v03x_finish_flag=0;                                                 //标志位清除
		}
		//************************图像处理****************************
		fsm_Event current_event=no_matter;
		int num=menu_num(current_state);
		//************************按键扫描****************************
		key_scanner();
		if(key_get_state(KEY_1) == KEY_SHORT_PRESS&&KEY_LONG_PRESS)
		{
			current_event=up;
			key_clear_state(KEY_1);
		}
		if(key_get_state(KEY_2) == KEY_SHORT_PRESS&&KEY_LONG_PRESS)
		{
			current_event=down;
			key_clear_state(KEY_2);
		}
		if(key_get_state(KEY_3) == KEY_SHORT_PRESS&&KEY_LONG_PRESS)
		{
			current_event=enter;
			key_clear_state(KEY_3);
		}
		if(key_get_state(KEY_4) == KEY_SHORT_PRESS&&KEY_LONG_PRESS)
		{
			current_event=esc;
			key_clear_state(KEY_4);
		}
		//****************************按键扫描*****************************
		//******************************菜单*******************************
		switch(current_state)
		{
			//***********************Main menu****************************
			case M_m:
				switch(current_event)
				{
					case up:
						current_p=(current_p+num-1)%num;
						main_menu_init();
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case down:
						current_p=(current_p+1)%num;
						main_menu_init();
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case enter:
						switch(current_p)
						{
							case 0://Motor Param
								current_state=M_Param;
								motor_param_menu_init();
								ips200_show_string(0,16,"->");
								current_p=0;
								break;
							case 1:
								current_state=I_mode;
								image_mode_menu_init();
								ips200_show_string(0,16,"->");
								current_p=0;
								break;
							case 2:
								current_state=D_m;
								break;
							default:
								break;
						}
					default:
						break;
				}
				break;
			//***********************Main menu****************************
			//*******************Motor Param menu*************************
			case M_Param:
				switch(current_event)
				{
					case up:
						current_p=(current_p+num-1)%num;
						motor_param_menu_init();
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case down:
						current_p=(current_p+1)%num;
						motor_param_menu_init();
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case enter:
						switch(current_p)
						{
							case 0://duty
								current_state=M_set_speed;
//								motor_duty_menu_init(duty);
//								current_p=0;
								break;
							case 1:
								current_state=y;
								break;
							default:
								break;
						}
						break;
					case esc:
						current_state=M_m;
						main_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;
			//*******************Motor Param menu**************************
			//**********************Motor Duty*****************************
//			case M_duty:
//				switch(current_event)
//				{
//					case up:
//						duty=(duty+100+1)%200-100;
//						motor_duty_menu_init(duty);
//						break;
//					case down:
//						duty=(duty+100-1)%200-100;
//						motor_duty_menu_init(duty);
//						break;
//					case esc:
//						current_state=M_Param;
//						motor_param_menu_init();
//						ips200_show_string(0,16,"->");
//						current_p=0;
//						break;
//					default:
//						break;
//				}
//				break;
			//**********************Motor Duty*****************************
			//**********************Image Mode*****************************
			case I_mode:
				switch(current_event)
				{
					case up:
						current_p=(current_p+num-1)%num;
						image_mode_menu_init();
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case down:
						current_p=(current_p+1)%num;
						image_mode_menu_init();
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case enter:
						switch(current_p)
						{
							case 1:                //Grey
								current_state=Grey_Im;
								ips200_clear();
								current_p=0;
								break;
							case 0:                //Binary
								current_state=Binary_Im;
								ips200_clear();
								current_p=0;
								break;
							default:
								break;
						}
						break;
					case esc:
						current_state=M_m;
						main_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;	
			//**********************Image Mode*****************************
			//**********************Grey image*****************************
			case Grey_Im:
				ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
				switch(current_event)
				{
					case esc:
						current_state=I_mode;
						image_mode_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;		
			//**********************Grey image*****************************
			//*********************Binary image****************************
			case Binary_Im:
				ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, Threshold);
				ips200_draw_line(90,0,90,70,RGB565_RED);
				for(int i=0;i<MT9V03X_H;i++)
				{
					ips200_draw_point((Left_Line[i]+Right_Line[i])>>1,i,RGB565_RED);
				}
				switch(current_event)
				{
					case esc:
						current_state=I_mode;
						image_mode_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;		
			//*********************Binary image****************************
			default:
				break;
		}
		//******************************菜单*******************************
		
		//*************************PID数据计算******************************
		
		PID_Calc(&Speed_PID,(float)(100-Speed_Real));
		PID_Calc(&Direction_PID,Err_Sum());
		
		//*************************PID数据计算******************************
		Left_duty=Left_duty+Speed_PID.output-Direction_PID.output;
		Right_duty=Left_duty+Speed_PID.output+Direction_PID.output;
		
		//******************************电机*******************************
		if(0 <= Left_duty)                                                           // 正转
        {
            gpio_set_level(DIR_L, GPIO_HIGH);                                   // DIR输出高电平
            pwm_set_duty(PWM_L, Left_duty * (PWM_DUTY_MAX / 100));              // 计算占空比
        }
        else                                                                    // 反转
        {
            gpio_set_level(DIR_L, GPIO_LOW);                                    // DIR输出低电平
            pwm_set_duty(PWM_L, (-Left_duty) * (PWM_DUTY_MAX / 100));                // 计算占空比
        }
		
		if(0 <= Right_duty)                                                           // 正转
        {
            gpio_set_level(DIR_R, GPIO_HIGH);                                   // DIR输出高电平
            pwm_set_duty(PWM_R, Right_duty * (PWM_DUTY_MAX / 100));              // 计算占空比
        }
        else                                                                    // 反转
        {
            gpio_set_level(DIR_R, GPIO_LOW);                                    // DIR输出低电平
            pwm_set_duty(PWM_R, (-Right_duty) * (PWM_DUTY_MAX / 100));                // 计算占空比
        }
		//******************************电机*******************************
		mt9v03x_finish_flag=0;
		system_delay_ms(50);
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    Speed_Left_Real = -encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    Speed_Right_Real = -encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
	
	Speed_Real = (Speed_Left_Real+Speed_Right_Real)/2;
}
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：S1-S4 按下 LED 不闪烁
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
//      万用表检查对应 S1-S4 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错
// 
// 问题2：SWITCH1 / SWITCH2 波动无 LED 反应
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
//      万用表检查对应 SWITCH1 / SWITCH2 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错
