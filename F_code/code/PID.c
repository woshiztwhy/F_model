#include "PID.h"
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     pid��ʼ��
  @param     null
  @return    null
  Sample     PID_Init(&mypid, 10, 1, 5, 800, 1000)
  @note      ��ʼ��kp,ki,kd,�Լ����ֺ�����޷�
-------------------------------------------------------------------------------------------------------------------*/
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����һ��pid����
  @param     PID�ṹ�壬�����
  @return    null
  Sample     PID_Calc(&Speed_PID,error)
  @note      ֱ�ӽ���������ֵ����ݼ�¼���ṹ��
-------------------------------------------------------------------------------------------------------------------*/
void PID_Calc(PID *pid , float new_error)
{
 	//��������
    pid->lastError = pid->error; //����error������
    pid->error = new_error; 
	
    //����΢��
    float dout = (pid->error - pid->lastError) * pid->kd;
	
    //�������
    float pout = pid->error * pid->kp;
	
    //�������
    pid->integral += pid->error * pid->ki;
	
    //�����޷�
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
	
    //�������
    pid->output = pout+dout + pid->integral;
	
    //����޷�
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}

 