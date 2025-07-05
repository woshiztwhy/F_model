#ifndef _image_deal_h
#define _image_deal_h

#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

extern  uint8    image_two_value[MT9V03X_H][MT9V03X_W];                          //二值化后图像数组

extern		const uint8  Standard_Road_Wide[MT9V03X_H];//标准赛宽数组
extern		volatile int Left_Line[MT9V03X_H]; //左边线数组
extern		volatile int Right_Line[MT9V03X_H];//右边线数组
extern		volatile int Mid_Line[MT9V03X_H];  //中线数组
extern		volatile int Road_Wide[MT9V03X_H]; //实际赛宽数组
extern		volatile int White_Column[MT9V03X_W];//每列白列长度
extern		volatile int Search_Stop_Line;     //搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
extern		volatile int Boundry_Start_Left;   //左右边界起始点
extern		volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
extern		volatile int Left_Lost_Time;       //边界丢线数
extern		volatile int Right_Lost_Time;
extern		volatile int Both_Lost_Time;//两边同时丢线数
extern		int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
extern		int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
extern		int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
extern		int Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0
                                                                                
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(int threshold);
void Longest_White_Column();

#endif

















