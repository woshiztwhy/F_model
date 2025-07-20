#include "image_deal.h"

uint8    image_two_value[MT9V03X_H][MT9V03X_W];

volatile int Left_Line[MT9V03X_H]; //左边线数组
volatile int Right_Line[MT9V03X_H];//右边线数组
volatile int Mid_Line[MT9V03X_H];  //中线数组
volatile int Road_Wide[MT9V03X_H]; //实际赛宽数组
volatile int White_Column[MT9V03X_W];//每列白列长度
volatile int Search_Stop_Line;     //搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
volatile int Boundry_Start_Left;   //左右边界起始点
volatile int Boundry_Start_Right;  //第一个非丢线点,常规边界起始点
volatile int Left_Lost_Time;       //边界丢线数
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//两边同时丢线数
int Longest_White_Column_Left[2]; //最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Longest_White_Column_Right[2];//最长白列,[0]是最长白列的长度，也就是Search_Stop_Line搜索截止行，[1】是第某列
int Left_Lost_Flag[MT9V03X_H] ; //左丢线数组，丢线置1，没丢线置0
int Right_Lost_Flag[MT9V03X_H]; //右丢线数组，丢线置1，没丢线置0

uint8 Right_Island_Flag=0;
uint8 Left_Island_Flag=0;
uint8 Island_State=0;
uint8 Cross_Flag=0;
uint8 Ramp_Flag=0;
uint8 Straight_Flag=0;
uint8 Zebra_Flag=0;


int Right_Down_Find;
int Left_Down_Find;
int Right_Up_Find;
int Left_Up_Find;

const uint8 Standard_Road_Wide[MT9V03X_H]={
	35,36,37,39,40,41,42,43,45,46,
	47,49,50,51,53,54,55,56,57,59,
	60,61,63,64,65,67,68,69,70,72,
	73,74,76,77,78,80,81,82,84,84,
	86,87,88,89,90,92,93,94,96,98,
	98,100,102,102,104,105,107,108,109,
	110,111,113,114,115,117,118,119,120,122
};

/*-------------------------------------------------------------------------------------------------------------------
  @brief     普通大津求阈值
  @param     image       图像数组
             width       列 ，宽度
             height      行，长度
  @return    threshold   返回int类型的的阈值
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//普通大津
-------------------------------------------------------------------------------------------------------------------*/
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height)   //大津算法，注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int  threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    for (i = 0; i < height; i+=2)//统计灰度级中每个像素在整幅图像中的个数
    {
        for (j = 0; j <width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    for (i = 0; i < GrayScale; i++) //计算每个像素值的点在整幅图像中的比例
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)//遍历灰度级[0,255]
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//平方
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;//最大类间方差法
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    if(threshold>255)
        threshold=255;
    if(threshold<0)
        threshold=0;
  return threshold;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     图像二值化处理函数
  @param     二值化阈值
  @return    NULL
  Sample     Image_Binarization(Threshold);//图像二值化
  @note      二值化后直接访问image_two_value[i][j]这个数组即可
-------------------------------------------------------------------------------------------------------------------*/
void Image_Binarization(int threshold)//图像二值化
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)//灰度图的数据只做判断，不进行更改，二值化图像放在了新数组中
        {
            if(mt9v03x_image[i][j]>=threshold)
                image_two_value[i][j]=IMG_WHITE;//白
            else
                image_two_value[i][j]=IMG_BLACK;//黑
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     二值化图像补中线函数
  @param     NULL
  @return    NULL
  Sample     Image_Add_Centerline();
  @note      二值化后直接访问image_two_value[i][j]这个数组即可
-------------------------------------------------------------------------------------------------------------------*/
void Image_Add_Centerline(void)
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        j=(Left_Line[i]+Right_Line[i])/2;
		image_two_value[i][j]=IMG_BLACK;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     二值化图像补边线函数
  @param     NULL
  @return    NULL
  Sample     Image_Add_Sideline();
  @note      二值化后直接访问image_two_value[i][j]这个数组即可
-------------------------------------------------------------------------------------------------------------------*/
void Image_Add_Sideline(void)
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        image_two_value[i][Left_Line[i]]=IMG_BLACK;
		image_two_value[i][Right_Line[i]]=IMG_BLACK;
    }
}
/*-------------------------------}
------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column()//最长白列巡线
{
    int i, j;
    int start_column=20;//最长白列的搜索区间
    int end_column=MT9V03X_W-20;
    int left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数
 
    for (i = 0; i <=MT9V03X_H-1; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = MT9V03X_W-1;
    }
    for(i=0;i<=MT9V03X_W-1;i++)
    {
        White_Column[i] = 0;
    }
 
////环岛需要对最长白列范围进行限定
//    //环岛3状态需要改变最长白列寻找范围
//    if(Right_Island_Flag==1)//右环
//    {
//        if(Island_State==3)
//        {
//            start_column=40;
//            end_column=MT9V03X_W-20;
//        }
//    }
//    else if(Left_Island_Flag==1)//左环
//    {
//        if(Island_State==3)
//        {
//            start_column=20;
//            end_column=MT9V03X_W-40;
//        }
//    }
 
    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)
    {
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if(image_two_value[i][j] == IMG_BLACK)
                break;
            else
                White_Column[j]++;
        }
    }
 
    //从左到右找左边最长白列
    Longest_White_Column_Left[0] =0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
        }
    }
    //从右到左找右左边最长白列
    Longest_White_Column_Right[0] = 0;//【0】是白列长度
    for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
        {
            Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
            Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
        }
    }
 
    Search_Stop_Line = Longest_White_Column_Left[0];//搜索截止行选取左或者右区别不大，他们两个理论上是一样的
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//常规巡线
    {
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//白黑黑，找到右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; //右丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j>=MT9V03X_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; //右丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//往左边扫描
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j - 1] == IMG_BLACK && image_two_value[i][j - 2] == IMG_BLACK)//黑黑白认为到达左边界
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; //左丢线数组，丢线置1，不丢线置0
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i] = 1; //左丢线数组，丢线置1，不丢线置0
                break;
            }
        }
        Left_Line [i] = left_border;       //左边线线数组
        Right_Line[i] = right_border;      //右边线线数组
    }
 
    for (i = MT9V03X_H - 1; i >= 0; i--)//赛道数据初步分析
    {
        if (Left_Lost_Flag[i]  == 1)//单边丢线数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//双边丢线数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//记录第一个非丢线点，边界起始点
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
 
//    //环岛3状态改变边界，看情况而定，我认为理论上的最优情况是不需要这些处理的
//    if(Island_State==3||Island_State==4)
//    {
//        if(Right_Island_Flag==1)//右环
//        {
//            for (i = MT9V03X_H - 1; i >= 0; i--)//右边直接写在边上
//            {
//                Right_Line[i]=MT9V03X_W-1;
//            }
//        }
//        else if(Left_Island_Flag==1)//左环
//        {
//            for (i = MT9V03X_H - 1; i >= 0; i--)//左边直接写在边上
//            {
//                Left_Line[i]=0;      //右边线线数组
//            }
//        }
//    }
      //debug使用，屏幕显示相关参数
//    ips200_showint16(0,0, Longest_White_Column_Right[0]);//【0】是白列长度
//    ips200_showint16(0,1, Longest_White_Column_Right[1]);//【1】是下标，第j列)
//    ips200_showint16(0,2, Longest_White_Column_Left[0]);//【0】是白列长度
//    ips200_showint16(0,3, Longest_White_Column_Left[1]);//【1】是下标，第j列)
}
//**********************************************************************************
/*-------------------------------------------------------------------------------------------------------------------
  @brief     误差计算
  @param     null
  @return    误差
  Sample     Err_Sum()；
  @note      通过中线计算误差，加权来减小干扰
-------------------------------------------------------------------------------------------------------------------*/
//加权控制
const uint8 Weight[MT9V03X_H]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端10 ——19 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端20 ——29 行权重
        1, 1, 5, 9,13,15,17,19,20,20,              //图像最远端30 ——39 行权重
       19,17,15,13,11, 9, 5, 1, 1, 1,              //图像最远端40 ——49 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端50 ——59 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端60 ——69 行权重
};
//误差计算
float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //常规误差
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line-1;i--)//常规误差计算
    {
        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];//右移1位，等效除2
        weight_count+=Weight[i];
    }
    err=err/weight_count;
    return err;//注意此处，误差有正负，还有小数，注意数据类型（右负左正）
}
//float Err_Sum(void)
//{
//    int i;
//    float err=0;
//    //常规误差
//     for(i=36;i<=40;i++)//36-40
//     {
//        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1));//右移1位，等效除2
//     }
//    err=err/5.0;
//    return err;//注意此处，误差有正负，还有小数，注意数据类型
//}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
     int i,a1,a2;
     int hx;
     if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
        x1=MT9V03X_W-1;
     else if(x1<=0)
         x1=0;
     if(y1>=MT9V03X_H-1)
         y1=MT9V03X_H-1;
     else if(y1<=0)
         y1=0;
     if(x2>=MT9V03X_W-1)
         x2=MT9V03X_W-1;
     else if(x2<=0)
              x2=0;
     if(y2>=MT9V03X_H-1)
         y2=MT9V03X_H-1;
     else if(y2<=0)
              y2=0;
     if(y1<y2)
	 {
		 a1=y1;
		 a2=y2;
	 }
	 else
	 {
		 a1=y2;
		 a2=y1;
	 }
     for(i=a1;i<=a2;i++)//根据斜率补线即可
     {
         hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
         if(hx>=MT9V03X_W)
             hx=MT9V03X_W;
         else if(hx<=0)
             hx=0;
         Left_Line[i]=hx;
     }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
     int i,a1,a2;
     int hx;
     if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
        x1=MT9V03X_W-1;
     else if(x1<=0)
         x1=0;
     if(y1>=MT9V03X_H-1)
         y1=MT9V03X_H-1;
     else if(y1<=0)
         y1=0;
     if(x2>=MT9V03X_W-1)
         x2=MT9V03X_W-1;
     else if(x2<=0)
              x2=0;
     if(y2>=MT9V03X_H-1)
         y2=MT9V03X_H-1;
     else if(y2<=0)
              y2=0;
     if(y1<y2)
	 {
		 a1=y1;
		 a2=y2;
	 }
	 else
	 {
		 a1=y2;
		 a2=y1;
	 }
     for(i=a1;i<=a2;i++)//根据斜率补线即可
     {
         hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
         if(hx>=MT9V03X_W)
             hx=MT9V03X_W;
         else if(hx<=0)
             hx=0;
         Right_Line[i]=hx;
     }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
 
    if(start<=5 && start <= end)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }
    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//这里的k是1/斜率
        if(start<=end)
        {
            for(i=start;i<=end;i++)
            {
                Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Left_Line[i]>=MT9V03X_W-1)
                {
                    Left_Line[i]=MT9V03X_W-1;
                }
                else if(Left_Line[i]<=0)
                {
                    Left_Line[i]=0;
                }
            }
        }
        else
        {
            for(i=end;i<=start;i++)
            {
                Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Left_Line[i]>=MT9V03X_W-1)
                {
                    Left_Line[i]=MT9V03X_W-1;
                }
                else if(Left_Line[i]<=0)
                {
                    Left_Line[i]=0;
                }
            }
        }
        
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
 
    if(start<=5 && start <= end)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;//这里的k是1/斜率
        if(start<=end)
        {
            for(i=start;i<=end;i++)
            {
                Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Right_Line[i]>=MT9V03X_W-1)
                {
                    Right_Line[i]=MT9V03X_W-1;
                }
                else if(Right_Line[i]<=0)
                {
                    Right_Line[i]=0;
                }
            }
        }
        else
        {
            for(i=end;i<=start;i++)
            {
                Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),点斜式变形
                if(Right_Line[i]>=MT9V03X_W-1)
                {
                    Right_Line[i]=MT9V03X_W-1;
                }
                else if(Right_Line[i]<=0)
                {
                    Right_Line[i]=0;
                }
            }
        }
        
    }
}
//斑马线检测
uint16 change=0;
void Zebra_Detect(void)
{
    int i,j;
    change=0;
    for(i=60;i>=45;i--)
    {
        for(j=15;j<=90;j++)
        {
            if(image_two_value[i][j+1]-image_two_value[i][j]!=0)
            {
                change++;
            }
        }
    }
   // lcd_showint16(100,6,change);
    if(change>=90)
    {
 
        Zebra_Flag=1;
    }
    else
    {
        Zebra_Flag=0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int Find_Right_Down_Point(int start,int end)//找四个角点，返回值是角点所在的行数
{
    int i,t;
    int right_down_line=0;
    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(right_down_line==0&&//只找第一个符合条件的点
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//角点的阈值可以更改
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&  
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-5&&
              (Right_Line[i]-Right_Line[i-3])<=-10&&
              (Right_Line[i]-Right_Line[i-4])<=-10)
        {
            right_down_line=i;//获取行数即可
            break;
        }
    }
    return right_down_line;
}