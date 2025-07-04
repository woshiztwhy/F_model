#ifndef _image_deal_h
#define _image_deal_h

#define IMG_BLACK     0X00      //0x00�Ǻ�
#define IMG_WHITE     0Xff      //0xffΪ��

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

extern  uint8    image_two_value[MT9V03X_H][MT9V03X_W];                          //��ֵ����ͼ������

extern	uint8    Longest_White_Column_Left[2];                                   //�����������,[0]Ϊ����,[1]Ϊ�±�
extern	uint8    Longest_White_Column_Right[2];									 //�����������,[0]Ϊ����,[1]Ϊ�±�
extern	uint8    Right_Lost_Time;                                                //���ߴ���
extern	uint8    Left_Lost_Time;                                                 //�Ҷ��ߴ���
extern	uint8    Boundry_Start_Left;                                             //�������ʼ��
extern	uint8    Boundry_Start_Right;                                            //�ұ�����ʼ��
extern	uint8    Both_Lost_Time;                                                 //�ܶ��ߴ���
extern	uint8    Right_Lost_Flag[MT9V03X_H];                                     //�Ҷ�������
extern	uint8    Left_Lost_Flag[MT9V03X_H];                                      //��������
extern	uint8    Left_Line[MT9V03X_H];                                           //���������
extern	uint8    Right_Line[MT9V03X_H];                                          //�ұ�������
extern	uint8    White_Column[MT9V03X_W];                                        //ÿ�а׵���������
extern	uint8    Right_Island_Flag;                                              //�󻷵���־
extern	uint8    Left_Island_Flag;                                               //�һ�����־
extern	uint8    Island_State;                                                   //����״̬
extern	uint8    Search_Stop_Line;                                               //����������
extern	uint8    Road_Wide[MT9V03X_H];                                           //�����ұ���ȷ����ÿ�������߿�                                
                                                                                 
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(int threshold);
void Longest_White_Column();

#endif