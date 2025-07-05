#ifndef _image_deal_h
#define _image_deal_h

#define IMG_BLACK     0X00      //0x00�Ǻ�
#define IMG_WHITE     0Xff      //0xffΪ��

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

extern  uint8    image_two_value[MT9V03X_H][MT9V03X_W];                          //��ֵ����ͼ������

extern		const uint8  Standard_Road_Wide[MT9V03X_H];//��׼��������
extern		volatile int Left_Line[MT9V03X_H]; //���������
extern		volatile int Right_Line[MT9V03X_H];//�ұ�������
extern		volatile int Mid_Line[MT9V03X_H];  //��������
extern		volatile int Road_Wide[MT9V03X_H]; //ʵ����������
extern		volatile int White_Column[MT9V03X_W];//ÿ�а��г���
extern		volatile int Search_Stop_Line;     //������ֹ��,ֻ��¼���ȣ���Ҫ������Ҫ����Ұ�߶ȼ�ȥ��ֵ
extern		volatile int Boundry_Start_Left;   //���ұ߽���ʼ��
extern		volatile int Boundry_Start_Right;  //��һ���Ƕ��ߵ�,����߽���ʼ��
extern		volatile int Left_Lost_Time;       //�߽綪����
extern		volatile int Right_Lost_Time;
extern		volatile int Both_Lost_Time;//����ͬʱ������
extern		int Longest_White_Column_Left[2]; //�����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
extern		int Longest_White_Column_Right[2];//�����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
extern		int Left_Lost_Flag[MT9V03X_H] ; //�������飬������1��û������0
extern		int Right_Lost_Flag[MT9V03X_H]; //�Ҷ������飬������1��û������0
                                                                                
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(int threshold);
void Longest_White_Column();

#endif

















