#include "image_deal.h"

uint8    image_two_value[MT9V03X_H][MT9V03X_W];

uint8    Longest_White_Column_Left[2];
uint8    Longest_White_Column_Right[2];
uint8    Right_Lost_Time;
uint8    Left_Lost_Time;
uint8    Boundry_Start_Left;
uint8    Boundry_Start_Right;
uint8    Both_Lost_Time;
uint8    Right_Lost_Flag[MT9V03X_H];
uint8    Left_Lost_Flag[MT9V03X_H];
uint8    Left_Line[MT9V03X_H];
uint8    Right_Line[MT9V03X_H];
uint8    White_Column[MT9V03X_W];
uint8    Right_Island_Flag;
uint8    Left_Island_Flag;
uint8    Island_State;
uint8    Search_Stop_Line;
uint8    Road_Wide[MT9V03X_H];
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ��ͨ�������ֵ
  @param     image       ͼ������
             width       �� �����
             height      �У�����
  @return    threshold   ����int���͵ĵ���ֵ
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//��ͨ���
-------------------------------------------------------------------------------------------------------------------*/
int My_Adapt_Threshold(uint8*image,uint16 width, uint16 height)   //����㷨��ע�������ֵ��һ��Ҫ��ԭͼ��
{
    #define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int  threshold = 0;
    uint8* data = image;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    for (i = 0; i < height; i+=2)//ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    {
        for (j = 0; j <width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }
    for (i = 0; i < GrayScale; i++) //����ÿ������ֵ�ĵ�������ͼ���еı���
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)//�����Ҷȼ�[0,255]
    {
        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
        u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //����ƽ���Ҷ�
        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//ƽ��
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;//�����䷽�
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
  @brief     ͼ���ֵ��������
  @param     ��ֵ����ֵ
  @return    NULL
  Sample     Image_Binarization(Threshold);//ͼ���ֵ��
  @note      ��ֵ����ֱ�ӷ���image_two_value[i][j]������鼴��
-------------------------------------------------------------------------------------------------------------------*/
void Image_Binarization(int threshold)//ͼ���ֵ��
{
    uint16 i,j;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)//�Ҷ�ͼ������ֻ���жϣ������и��ģ���ֵ��ͼ���������������
        {
            if(mt9v03x_image[i][j]>=threshold)
                image_two_value[i][j]=IMG_WHITE;//��
            else
                image_two_value[i][j]=IMG_BLACK;//��
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ˫�����Ѳ��
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      �����Ѳ�ߣ�Ѱ�ҳ�ʼ�߽磬���ߣ�����еȻ���Ԫ�أ�������ȡ��Щ��������������ʶ��
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column()//�����Ѳ��
{
    int i, j;
    int start_column=20;//����е���������
    int end_column=MT9V03X_W-20;
    int left_border = 0, right_border = 0;//��ʱ�洢����λ��
    Longest_White_Column_Left[0] = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Left[1] = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Right[0] = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Longest_White_Column_Right[1] = 0;//�����,[0]������еĳ��ȣ�[1���ǵ�ĳ��
    Right_Lost_Time = 0;    //�߽綪����
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//��һ���Ƕ��ߵ�,����߽���ʼ��
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//����ͬʱ������
 
    for (i = 0; i <=MT9V03X_H-1; i++)//��������
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
 
//������Ҫ������з�Χ�����޶�
    //����3״̬��Ҫ�ı������Ѱ�ҷ�Χ
    if(Right_Island_Flag==1)//�һ�
    {
        if(Island_State==3)
        {
            start_column=40;
            end_column=MT9V03X_W-20;
        }
    }
    else if(Left_Island_Flag==1)//��
    {
        if(Island_State==3)
        {
            start_column=20;
            end_column=MT9V03X_W-40;
        }
    }
 
    //�����ң��������ϣ�����ȫͼ��¼��Χ�ڵ�ÿһ�а׵�����
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
 
    //����������������
    Longest_White_Column_Left[0] =0;
    for(i=start_column;i<=end_column;i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])//�������һ��
        {
            Longest_White_Column_Left[0] = White_Column[i];//��0���ǰ��г���
            Longest_White_Column_Left[1] = i;              //��1�����±꣬��j��
        }
    }
    //���ҵ���������������
    Longest_White_Column_Right[0] = 0;//��0���ǰ��г���
    for(i=end_column;i>=start_column;i--)//��������ע���������ҵ���������λ�þͿ���ͣ��
    {
        if (Longest_White_Column_Right[0] < White_Column[i])//�������һ��
        {
            Longest_White_Column_Right[0] = White_Column[i];//��0���ǰ��г���
            Longest_White_Column_Right[1] = i;              //��1�����±꣬��j��
        }
    }
 
    Search_Stop_Line = Longest_White_Column_Left[0];//������ֹ��ѡȡ����������𲻴�����������������һ����
    for (i = MT9V03X_H - 1; i >=MT9V03X_H-Search_Stop_Line; i--)//����Ѳ��
    {
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//�׺ںڣ��ҵ��ұ߽�
            {
                right_border = j;
                Right_Lost_Flag[i] = 0; //�Ҷ������飬������1����������0
                break;
            }
            else if(j>=MT9V03X_W-1-2)//û�ҵ��ұ߽磬����Ļ���Ҹ�ֵ���ұ߽�
            {
                right_border = j;
                Right_Lost_Flag[i] = 1; //�Ҷ������飬������1����������0
                break;
            }
        }
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//�����ɨ��
        {
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j - 1] == IMG_BLACK && image_two_value[i][j - 2] == IMG_BLACK)//�ںڰ���Ϊ������߽�
            {
                left_border = j;
                Left_Lost_Flag[i] = 0; //�������飬������1����������0
                break;
            }
            else if(j<=0+2)
            {
                left_border = j;//�ҵ�ͷ��û�ҵ��ߣ��Ͱ���Ļ�����ҵ����߽�
                Left_Lost_Flag[i] = 1; //�������飬������1����������0
                break;
            }
        }
        Left_Line [i] = left_border;       //�����������
        Right_Line[i] = right_border;      //�ұ���������
    }
 
    for (i = MT9V03X_H - 1; i >= 0; i--)//�������ݳ�������
    {
        if (Left_Lost_Flag[i]  == 1)//���߶�����
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)//˫�߶�����
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]  != 1)//��¼��һ���Ƕ��ߵ㣬�߽���ʼ��
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
    }
 
    //����3״̬�ı�߽磬���������������Ϊ�����ϵ���������ǲ���Ҫ��Щ�����
    if(Island_State==3||Island_State==4)
    {
        if(Right_Island_Flag==1)//�һ�
        {
            for (i = MT9V03X_H - 1; i >= 0; i--)//�ұ�ֱ��д�ڱ���
            {
                Right_Line[i]=MT9V03X_W-1;
            }
        }
        else if(Left_Island_Flag==1)//��
        {
            for (i = MT9V03X_H - 1; i >= 0; i--)//���ֱ��д�ڱ���
            {
                Left_Line[i]=0;      //�ұ���������
            }
        }
    }
      //debugʹ�ã���Ļ��ʾ��ز���
//    ips200_showint16(0,0, Longest_White_Column_Right[0]);//��0���ǰ��г���
//    ips200_showint16(0,1, Longest_White_Column_Right[1]);//��1�����±꣬��j��)
//    ips200_showint16(0,2, Longest_White_Column_Left[0]);//��0���ǰ��г���
//    ips200_showint16(0,3, Longest_White_Column_Left[1]);//��1�����±꣬��j��)
}