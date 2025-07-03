#ifndef _image_deal_h
#define _image_deal_h

#define IMG_BLACK     0X00      //0x00是黑
#define IMG_WHITE     0Xff      //0xff为白

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

extern uint8    image_two_value[MT9V03X_H][MT9V03X_W];                          //二值化后图像数组

int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height);
void Image_Binarization(int threshold);

#endif