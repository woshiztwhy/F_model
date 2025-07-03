#ifndef _image_deal_h
#define _image_deal_h

#define IMG_BLACK     0X00      //0x00�Ǻ�
#define IMG_WHITE     0Xff      //0xffΪ��

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

extern uint8    image_two_value[MT9V03X_H][MT9V03X_W];                          //��ֵ����ͼ������

int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(int threshold);

#endif