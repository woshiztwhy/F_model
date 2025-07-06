#include "PID.h"
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     pid初始化
  @param     null
  @return    null
  Sample     PID_Init(&mypid, 10, 1, 5, 800, 1000)
  @note      初始化kp,ki,kd,以及积分和输出限幅
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
  @brief     进行一次pid运算
  @param     PID结构体，新误差
  @return    null
  Sample     PID_Calc(&Speed_PID,error)
  @note      直接将输出，积分等数据记录给结构体
-------------------------------------------------------------------------------------------------------------------*/
void PID_Calc(PID *pid , float new_error)
{
 	//更新数据
    pid->lastError = pid->error; //将旧error存起来
    pid->error = new_error; 
	
    //计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;
	
    //计算比例
    float pout = pid->error * pid->kp;
	
    //计算积分
    pid->integral += pid->error * pid->ki;
	
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
	
    //计算输出
    pid->output = pout+dout + pid->integral;
	
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}

 