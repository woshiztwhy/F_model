#include "image_deal.h"

uint8    image_two_value[MT9V03X_H][MT9V03X_W];

const uint8  Standard_Road_Wide[MT9V03X_H];//��׼��������
volatile int Left_Line[MT9V03X_H]; //���������
volatile int Right_Line[MT9V03X_H];//�ұ�������
volatile int Mid_Line[MT9V03X_H];  //��������
volatile int Road_Wide[MT9V03X_H]; //ʵ����������
volatile int White_Column[MT9V03X_W];//ÿ�а��г���
volatile int Search_Stop_Line;     //������ֹ��,ֻ��¼���ȣ���Ҫ������Ҫ����Ұ�߶ȼ�ȥ��ֵ
volatile int Boundry_Start_Left;   //���ұ߽���ʼ��
volatile int Boundry_Start_Right;  //��һ���Ƕ��ߵ�,����߽���ʼ��
volatile int Left_Lost_Time;       //�߽綪����
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;//����ͬʱ������
int Longest_White_Column_Left[2]; //�����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
int Longest_White_Column_Right[2];//�����,[0]������еĳ��ȣ�Ҳ����Search_Stop_Line������ֹ�У�[1���ǵ�ĳ��
int Left_Lost_Flag[MT9V03X_H] ; //�������飬������1��û������0
int Right_Lost_Flag[MT9V03X_H]; //�Ҷ������飬������1��û������0

uint8 Right_Island_Flag=0;
uint8 Left_Island_Flag=0;
uint8 Island_State=0;
uint8 Cross_Flag=0;
uint8 Ramp_Flag=0;
uint8 Straight_Flag=0;


int Right_Down_Find;
int Left_Down_Find;
int Right_Up_Find;
int Left_Up_Find;

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
//**********************************************************************************
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������
  @param     null
  @return    ���
  Sample     Err_Sum()��
  @note      ͨ�����߼�������Ȩ����С����
-------------------------------------------------------------------------------------------------------------------*/
//��Ȩ����
const uint8 Weight[MT9V03X_H]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��00 ����09 ��Ȩ��
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��10 ����19 ��Ȩ��
        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,              //ͼ����Զ��20 ����29 ��Ȩ��
        6, 7, 9,11,13,15,17,19,20,20,              //ͼ����Զ��30 ����39 ��Ȩ��
       19,17,15,13,11, 9, 7, 5, 3, 1,              //ͼ����Զ��40 ����49 ��Ȩ��
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��50 ����59 ��Ȩ��
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //ͼ����Զ��60 ����69 ��Ȩ��
};
//������
float Err_Sum(void)
{
    int i;
    float err=0;
    float weight_count=0;
    //�������
    for(i=MT9V03X_H-1;i>=MT9V03X_H-Search_Stop_Line-1;i--)//����������
    {
        err+=(MT9V03X_W/2-((Left_Line[i]+Right_Line[i])>>1))*Weight[i];//����1λ����Ч��2
        weight_count+=Weight[i];
    }
    err=err/weight_count;
    return err;//ע��˴������������������С����ע���������ͣ��Ҹ�������
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ֱ�����
  @param     null
  @return    null
  Sample     Straight_Detect(err)��
  @note      ��������У��߽���ʼ�㣬������ʼ�㣬
-------------------------------------------------------------------------------------------------------------------*/
void Straight_Detect(float Err)
{
    Straight_Flag=0;
    if(Search_Stop_Line>=65)//��ֹ�к�Զ
    {
        if(Boundry_Start_Left>=68&&Boundry_Start_Right>=65)//��ʼ�㿿��
        {
            if(-5<=Err&&Err<=5)//����С
            {
                Straight_Flag=1;//��Ϊ��ֱ��
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int start,int end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int start,int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-5)//����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=MT9V03X_H-1-5;
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(Left_Line[i]-Left_Line[i+1])<=5&&//�ǵ����ֵ���Ը���
           abs(Left_Line[i+1]-Left_Line[i+2])<=5&&
           abs(Left_Line[i+2]-Left_Line[i+3])<=5&&
              (Left_Line[i]-Left_Line[i-2])>=8&&
              (Left_Line[i]-Left_Line[i-3])>=15&&
              (Left_Line[i]-Left_Line[i-4])>=15)
        {
            Left_Down_Find=i;//��ȡ��������
        }
        if(Right_Down_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(Right_Line[i]-Right_Line[i+1])<=5&&//�ǵ����ֵ���Ը���
           abs(Right_Line[i+1]-Right_Line[i+2])<=5&&
           abs(Right_Line[i+2]-Right_Line[i+3])<=5&&
              (Right_Line[i]-Right_Line[i-2])<=-8&&
              (Right_Line[i]-Right_Line[i-3])<=-15&&
              (Right_Line[i]-Right_Line[i-4])<=-15)
        {
            Right_Down_Find=i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//�����ҵ����˳�
        {
            break;
        }
    }
}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int start,int end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int start,int end)
{
    int i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=MT9V03X_H-Search_Stop_Line)
        end=MT9V03X_H-Search_Stop_Line;
    if(end<=5)//��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��
        end=5;
    if(start>=MT9V03X_H-1-5)//����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=MT9V03X_H-1-5;
    for(i=start;i>=end;i--)
    {
        if(Left_Up_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(Left_Line[i]-Left_Line[i-1])<=5&&
           abs(Left_Line[i-1]-Left_Line[i-2])<=5&&
           abs(Left_Line[i-2]-Left_Line[i-3])<=5&&
              (Left_Line[i]-Left_Line[i+2])>=8&&
              (Left_Line[i]-Left_Line[i+3])>=15&&
              (Left_Line[i]-Left_Line[i+4])>=15)
        {
            Left_Up_Find=i;//��ȡ��������
        }
        if(Right_Up_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(Right_Line[i]-Right_Line[i-1])<=5&&//��������λ�ò��
           abs(Right_Line[i-1]-Right_Line[i-2])<=5&&
           abs(Right_Line[i-2]-Right_Line[i-3])<=5&&
              (Right_Line[i]-Right_Line[i+2])<=-8&&
              (Right_Line[i]-Right_Line[i+3])<=-15&&
              (Right_Line[i]-Right_Line[i+4])<=-15)
        {
            Right_Up_Find=i;//��ȡ��������
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//���������ҵ��ͳ�ȥ
        {
            break;
        }
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//����˺�ѹ�����Ϊ����
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����
  @param     ���ߵ���㣬�յ�
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      ����ֱ���Ǳ߽磬������ǿ��Ŷȸߵ�,��Ҫ�Ҳ�
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//����,�����Ǳ߽�
{
     int i,a1,a2;
     int hx;
     if(x1>=MT9V03X_W-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���
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
     for(i=a1;i<=a2;i++)//����б�ʲ��߼���
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
  @brief     �Ҳ���
  @param     ���ߵ���㣬�յ�
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      ����ֱ���Ǳ߽磬������ǿ��Ŷȸߵ�,��Ҫ�Ҳ�
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//����,�����Ǳ߽�
{
     int i,a1,a2;
     int hx;
     if(x1>=MT9V03X_W-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���
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
     for(i=a1;i<=a2;i++)//����б�ʲ��߼���
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
  @brief     ��߽��ӳ�
  @param     �ӳ���ʼ�������ӳ���ĳ��
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)��
  @note      ����ʼ��������3���㣬���б�ʣ������ӳ���ֱ��������
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
 
    if(start<=5 && start <= end)//��Ϊ��Ҫ�ڿ�ʼ��������3���㣬������ʼ����ڿ��ϣ��������ӳ���ֻ��ֱ������
    {
        Left_Add_Line(Left_Line[start],start,Left_Line[end],end);
    }
    else
    {
        k=(float)(Left_Line[start]-Left_Line[start-4])/5.0;//�����k��1/б��
        if(start<=end)
        {
            for(i=start;i<=end;i++)
            {
                Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),��бʽ����
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
                Left_Line[i]=(int)(i-start)*k+Left_Line[start];//(x=(y-y1)*k+x1),��бʽ����
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
  @brief     �ұ߽��ӳ�
  @param     �ӳ���ʼ�������ӳ���ĳ��
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)��
  @note      ����ʼ��������3���㣬���б�ʣ������ӳ���ֱ��������
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
 
    if(start<=5 && start <= end)//��Ϊ��Ҫ�ڿ�ʼ��������3���㣬������ʼ����ڿ��ϣ��������ӳ���ֻ��ֱ������
    {
        Right_Add_Line(Right_Line[start],start,Right_Line[end],end);
    }
    else
    {
        k=(float)(Right_Line[start]-Right_Line[start-4])/5.0;//�����k��1/б��
        if(start<=end)
        {
            for(i=start;i<=end;i++)
            {
                Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),��бʽ����
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
                Right_Line[i]=(int)(i-start)*k+Right_Line[start];//(x=(y-y1)*k+x1),��бʽ����
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
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ʮ�ּ��
  @param     null
  @return    null
  Sample     Cross_Detect()��
  @note      ���ùյ㲹�ߣ����ʮ��
-------------------------------------------------------------------------------------------------------------------*/
void Cross_Detect()
{
    int down_search_start=0;//�½ǵ�������ʼ��
    Cross_Flag=0;
    if(Island_State==0&&Ramp_Flag==0)//�뻷�����⿪
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time>=10)//ʮ�ֱض���˫�߶��ߣ�����˫�߶��ߵ�������ٿ�ʼ�ҽǵ�
        {
            Find_Up_Point( MT9V03X_H-1, 0 );
            if(Left_Up_Find==0&&Right_Up_Find==0)//ֻҪû��ͬʱ�ҵ������ϵ㣬ֱ�ӽ���
            {
                return;
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//�ҵ������ϵ㣬����Ϊ�ҵ�ʮ����
        {
            Cross_Flag=1;//ȷ����Ӧ��־λ�����ڸ�Ԫ�ػ����
            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//�������Ϲյ����꿿������Ϊ�µ����������
            Find_Down_Point(MT9V03X_H-5,down_search_start+2);//���Ϲյ���2����Ϊ�½ǵ�Ľ�ֹ��
            if(Left_Down_Find<=Left_Up_Find)
            {
                Left_Down_Find=0;//�µ㲻���ܱ��ϵ㻹����
            }
            if(Right_Down_Find<=Right_Up_Find)
            {
                Right_Down_Find=0;//�µ㲻���ܱ��ϵ㻹����
            }
            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//�ĸ��㶼�ڣ��������ߣ����������Ȼ����
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//����ʹ�õ���б�ʲ���
            {//������                                     //01
                Lengthen_Left_Boundry(Left_Up_Find-1,MT9V03X_H-1);
                Right_Add_Line(Right_Line[Right_Up_Find],Right_Up_Find,Right_Line[Right_Down_Find],Right_Down_Find);
            }
            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
            {//������                                      //10
                Left_Add_Line (Left_Line [Left_Up_Find ],Left_Up_Find ,Left_Line [Left_Down_Find ] ,Left_Down_Find);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
            {//�����ϵ�                                    //00
                Lengthen_Left_Boundry (Left_Up_Find-1,MT9V03X_H-1);
                Lengthen_Right_Boundry(Right_Up_Find-1,MT9V03X_H-1);
            }
        }
        else
        {
            Cross_Flag=0;
        }
    }
    //�ǵ���ر�����debugʹ��
    //ips200_showuint8(0,12,Cross_Flag);
//    ips200_showuint8(0,13,Island_State);
//    ips200_showuint8(50,12,Left_Up_Find);
//    ips200_showuint8(100,12,Right_Up_Find);
//    ips200_showuint8(50,13,Left_Down_Find);
//    ips200_showuint8(100,13,Right_Down_Find);
}