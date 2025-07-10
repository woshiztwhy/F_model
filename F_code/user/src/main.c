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
#define LED1                    (H2 )

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

#define FLASH_SECTION_INDEX       (127)                                         // 存储数据用的扇区 倒数第一个扇区
#define FLASH_PAGE_INDEX          (3)                                           // 存储数据用的页码 倒数第一个页码

void flash_data_buffer_printf (void);

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

uint32 time_count=0;

fsm_State current_state=M_m;
uint8 current_p=0;

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
	
	encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                     // 初始化编码器模块与引脚 
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);                     // 初始化编码器模块与引脚 
    pit_ms_init(PIT, 10);
	
	PID_Init(&Direction_PID,0,0,0,8,10);
	PID_Init(&Speed_PID,0,0,0,1,10);
	
	//*********************flash数据读入**********************
//	if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
//    {
//        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                // 擦除这一页
//    }
	flash_buffer_clear();    
	flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区
	Speed_PID.kp=flash_union_buffer[0].float_type;
	Speed_PID.ki=flash_union_buffer[1].float_type;
	Speed_PID.kd=flash_union_buffer[2].float_type;
	Direction_PID.kp=flash_union_buffer[3].float_type;
	Direction_PID.ki=flash_union_buffer[4].float_type;
	Direction_PID.kd=flash_union_buffer[5].float_type;
	
//	printf("\r\n float_type : %f", flash_union_buffer[0].float_type);
	//*********************flash数据读入**********************
	//********************无线串口初始化**********************
	gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
    if(wireless_uart_init())                                                    // 判断是否通过初始化
    {
        while(1)                                                                // 初始化失败就在这进入死循环
        {
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
            system_delay_ms(100);                                               // 短延时快速闪灯表示异常
        }
    }
	//*********************无线串口初始化**********************
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
	
//	printf("\r\n %d",Speed_Set);
	main_menu_init();
	ips200_show_string(0,16,"->");
	
    while(1)
	{
		//************************图像处理****************************
		if(mt9v03x_finish_flag)                                                    //先处理图像，再清除标志位
		{
			Threshold=My_Adapt_Threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);   //大津算阈值
			Image_Binarization(Threshold);                                         //图像二值化
			Longest_White_Column();                                                //最长白线法寻边线
			Cross_Detect();                                                        //十字检测补线
			Image_Add_Centerline();                                                //二值化图像补中线
			Image_Add_Sideline();                                                  //二值化图像补边线
			mt9v03x_finish_flag=0;
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
				pwm_set_duty(PWM_L, 0);
				pwm_set_duty(PWM_R, 0);
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
							case 1://Image mode
								current_state=I_mode;
								image_mode_menu_init();
								ips200_show_string(0,16,"->");
								current_p=0;
								break;
							case 2:
								current_state=D_m;
								departure_mode_init();
								system_delay_ms(1500);
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
							case 0:
								current_state=Dir_PID_State;
								motor_dir_pid_init(&Direction_PID);
								ips200_show_string(0,16,"->");
								current_p=0;
								break;
							case 1:
								current_state=S_PID_State;
								motor_speed_pid_init(&Speed_PID);
								ips200_show_string(0,16,"->");
								current_p=0;
								break;
							case 2:
								current_state=M_set_speed;
								motor_set_speed_menu_init(Speed_Set);
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
			//*******************Motor Param menu**************************
			//*******************Motor Set Speed***************************
			case M_set_speed:
				switch(current_event)
				{
					case up:
						Speed_Set++;
						motor_set_speed_menu_init(Speed_Set);
						break;
					case down:
						Speed_Set--;
						motor_set_speed_menu_init(Speed_Set);
						break;
					case esc:
						current_state=M_Param;
						motor_param_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;
			//*******************Motor Set Speed***************************
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
					ips200_draw_point(Left_Line[i],i,RGB565_RED);
					ips200_draw_point(Right_Line[i],i ,RGB565_RED);
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
			//********************Depature  mode***************************
			case D_m:
				if(Longest_White_Column_Left[0]<=MT9V03X_H/10)
				{
					current_state=M_m;
					main_menu_init();
					ips200_show_string(0,16,"->");
					current_p=0;
					break;
				}
//				if(mt9v03x_finish_flag)                                                    //先处理图像，再清除标志位
//				{
//					camera_send_image(WIRELESS_UART_INDEX, (const uint8 *)image_two_value, MT9V03X_IMAGE_SIZE);
//					mt9v03x_finish_flag=0;
//				}
				printf("\r\n %d,%d",Speed_Set,Speed_Real);
				ips200_show_int(48,16,Speed_Real,4);
				ips200_show_float(54,48,Speed_PID.output,4,4);
				//******************************电机*******************************
				if(Left_duty>MAX_DUTY)
				{
					Left_duty=MAX_DUTY;
				}
				if(Right_duty>MAX_DUTY)
				{
					Right_duty=MAX_DUTY;
				}
				if(Left_duty<-MAX_DUTY)
				{
					Left_duty=-MAX_DUTY;
				}
				if(Right_duty<-MAX_DUTY)
				{
					Right_duty=-MAX_DUTY;
				}
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
				switch(current_event)
				{
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
			//********************Depature  mode***************************
			//***********************Speed PID*****************************
			case S_PID_State:
				switch(current_event)
				{
					case up:
						current_p=(current_p+num-1)%num;
						motor_speed_pid_init(&Speed_PID);
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case down:
						current_p=(current_p+1)%num;
						motor_speed_pid_init(&Speed_PID);
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case enter:
						switch(current_p)
						{
							case 0:                
								current_state=Speed_p;
								current_p=0;
								break;
							case 1:                
								current_state=Speed_i;
								current_p=0;
								break;
							case 2:                
								current_state=Speed_d;
								current_p=0;
								break;
							default:
								break;
						}
						break;
					case esc:
						current_state=M_Param;
						motor_param_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;	
			//***********************Speed PID*****************************
			//***********************速度p调节******************************
			case Speed_p:
				switch(current_event)
				{
					case up:
						Speed_PID.kp+=0.001;
						motor_speed_pid_init(&Speed_PID);
						break;
					case down:
						Speed_PID.kp-=0.001;
						motor_speed_pid_init(&Speed_PID);
						break;
					case esc:
						current_state=S_PID_State;
						motor_speed_pid_init(&Speed_PID);
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					case enter:
						//***************************flash 写入****************************
						flash_buffer_clear();
						flash_union_buffer[0].float_type=Speed_PID.kp;
						flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
						//***************************flash 写入****************************
						break;
					default:
						break;
				}
				break;
			//***********************速度p调节******************************
			//***********************速度i调节******************************
			case Speed_i:
				switch(current_event)
				{
					case up:
						Speed_PID.ki+=0.001;
						motor_speed_pid_init(&Speed_PID);
						break;
					case down:
						Speed_PID.ki-=0.001;
						motor_speed_pid_init(&Speed_PID);
						break;
					case esc:
						current_state=S_PID_State;
						motor_speed_pid_init(&Speed_PID);
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					case enter:
						//***************************flash 写入****************************
						flash_buffer_clear();
						flash_union_buffer[1].float_type=Speed_PID.ki;
						flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
						//***************************flash 写入****************************
						break;
					default:
						break;
				}
				break;
			//***********************速度i调节******************************
			//***********************速度d调节******************************
			case Speed_d:
				switch(current_event)
				{
					case up:
						Speed_PID.kd+=0.001;
						motor_speed_pid_init(&Speed_PID);
						break;
					case down:
						Speed_PID.kd-=0.001;
						motor_speed_pid_init(&Speed_PID);
						break;
					case esc:
						current_state=S_PID_State;
						motor_speed_pid_init(&Speed_PID);
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					case enter:
						//***************************flash 写入****************************
						flash_buffer_clear();
						flash_union_buffer[2].float_type=Speed_PID.kd;
						flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
						//***************************flash 写入****************************
						break;
					default:
						break;
				}
				break;
			//***********************速度d调节******************************
			//************************Dir PID*****************************
			case Dir_PID_State:
				switch(current_event)
				{
					case up:
						current_p=(current_p+num-1)%num;
						motor_dir_pid_init(&Direction_PID);
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case down:
						current_p=(current_p+1)%num;
						motor_dir_pid_init(&Direction_PID);
						ips200_show_string(0,(current_p+1)*16,"->");
						break;
					case enter:
						switch(current_p)
						{
							case 0:                
								current_state=Dir_p;
								current_p=0;
								break;
							case 1:                
								current_state=Dir_i;
								current_p=0;
								break;
							case 2:                
								current_state=Dir_d;
								current_p=0;
								break;
							default:
								break;
						}
						break;
					case esc:
						current_state=M_Param;
						motor_param_menu_init();
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					default:
						break;
				}
				break;	
			//************************Dir PID*****************************
			//***********************方向p调节******************************
			case Dir_p:
				switch(current_event)
				{
					case up:
						Direction_PID.kp+=0.001;
						motor_dir_pid_init(&Direction_PID);
						break;
					case down:
						Direction_PID.kp-=0.001;
						motor_dir_pid_init(&Direction_PID);
						break;
					case esc:
						current_state=Dir_PID_State;
						motor_dir_pid_init(&Direction_PID);
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					case enter:
						//***************************flash 写入****************************
						flash_buffer_clear();
						flash_union_buffer[3].float_type=Direction_PID.kp;
						flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
						//***************************flash 写入****************************
						break;
					default:
						break;
				}
				break;
			//***********************方向p调节******************************
			//***********************方向i调节******************************
			case Dir_i:
				switch(current_event)
				{
					case up:
						Direction_PID.ki+=0.001;
						motor_dir_pid_init(&Direction_PID);
						break;
					case down:
						Direction_PID.ki-=0.001;
						motor_dir_pid_init(&Direction_PID);
						break;
					case esc:
						current_state=Dir_PID_State;
						motor_dir_pid_init(&Direction_PID);
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					case enter:
						//***************************flash 写入****************************
						flash_buffer_clear();
						flash_union_buffer[4].float_type=Direction_PID.ki;
						flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
						//***************************flash 写入****************************
						break;
					default:
						break;
				}
				break;
			//***********************方向i调节******************************
			//***********************方向d调节******************************
			case Dir_d:
				switch(current_event)
				{
					case up:
						Direction_PID.kd+=0.001;
						motor_dir_pid_init(&Direction_PID);
						break;
					case down:
						Direction_PID.kd-=0.001;
						motor_dir_pid_init(&Direction_PID);
						break;
					case esc:
						current_state=Dir_PID_State;
						motor_dir_pid_init(&Direction_PID);
						ips200_show_string(0,16,"->");
						current_p=0;
						break;
					case enter:
						//***************************flash 写入****************************
						flash_buffer_clear();
						flash_union_buffer[5].float_type=Direction_PID.kd;
						flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
						//***************************flash 写入****************************
						break;
					default:
						break;
				}
				break;
			//***********************方向d调节******************************
			default:
				break;
			
		}
		//******************************菜单*******************************
//		flash_buffer_clear();
//		flash_union_buffer[0].float_type=0;
//		flash_union_buffer[1].float_type=0;
//		flash_union_buffer[2].float_type=0;
//		flash_union_buffer[3].float_type=0;
//		flash_union_buffer[4].float_type=0;
//		flash_union_buffer[5].float_type=0;
//		flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
		mt9v03x_finish_flag=0;
		system_delay_ms(5);
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
    Speed_Left_Real = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    Speed_Right_Real = -encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数
	
	Speed_Real = (Speed_Left_Real+Speed_Right_Real)/2;
	
	//*************************PID数据计算******************************
		
	PID_Calc(&Speed_PID,(float)(Speed_Set-Speed_Real));
	PID_Calc(&Direction_PID,Err_Sum());
	
	//*************************PID数据计算******************************
	Left_duty=Left_duty+Speed_PID.output-Direction_PID.output;
	Right_duty=Left_duty+Speed_PID.output+Direction_PID.output;
	
//	if(current_state==D_m)           //定时关闭电机
//	{
//		time_count++;
//		if(time_count>300)
//		{
//			current_state=M_m;
//			main_menu_init();
//			ips200_show_string(0,16,"->");
//			current_p=0;
//			time_count=0;
//		}
//	}                             
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
