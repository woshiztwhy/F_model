#ifndef _image_deal_h
#define _image_deal_h

#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

extern  uint8    image_two_value[MT9V03X_H][MT9V03X_W];                          //二值化后图像数组

extern	uint8    Longest_White_Column_Left[2];                                   //左最长白列数据,[0]为长度,[1]为下标
extern	uint8    Longest_White_Column_Right[2];									 //右最长白列数据,[0]为长度,[1]为下标
extern	uint8    Right_Lost_Time;                                                //左丢线次数
extern	uint8    Left_Lost_Time;                                                 //右丢线次数
extern	uint8    Boundry_Start_Left;                                             //左边线起始点
extern	uint8    Boundry_Start_Right;                                            //右边线起始点
extern	uint8    Both_Lost_Time;                                                 //总丢线次数
extern	uint8    Right_Lost_Flag[MT9V03X_H];                                     //右丢线数组
extern	uint8    Left_Lost_Flag[MT9V03X_H];                                      //左丢线数组
extern	uint8    Left_Line[MT9V03X_H];                                           //左边线数组
extern	uint8    Right_Line[MT9V03X_H];                                          //右边线数组
extern	uint8    White_Column[MT9V03X_W];                                        //每列白点数量数组
extern	uint8    Right_Island_Flag;                                              //左环岛标志
extern	uint8    Left_Island_Flag;                                               //右环岛标志
extern	uint8    Island_State;                                                   //环岛状态
extern	uint8    Search_Stop_Line;                                               //搜索截至行
extern	uint8    Road_Wide[MT9V03X_H];                                           //由左右边线确定的每行理论线宽                                
                                                                                 
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(int threshold);
void Longest_White_Column();

#endif