#include "pid.h"
#include <stdio.h>
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[8], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->Bas_KP = PID[3];
    pid->Gain_KP = PID[4];
    pid->Max_I = PID[5];
    pid->Cp = PID[6];
    pid->Ci = PID[7];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid????
  * @param[out]     pid: PID?????????
  * @param[in]      ref: ????????
  * @param[in]      set: ???
  * @retval         pid???
  */

fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1]; //上上次误差
    pid->error[1] = pid->error[0]; //上次误差
    pid->set = set;
    pid->fdb = ref;                 //反馈
    pid->error[0] = set - ref; //当前误差

    if (pid->mode == PID_POSITION) //位置式
    {
        pid->Pout = pid->Kp * pid->error[0];
				if(fabs(pid->error[0])<250){
					pid->Iout += pid->Ki * pid->error[0];
				}else{
					pid->Iout = 0;
				}
				pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);
				LimitMax(pid->Iout,pid->max_iout)
        pid->out = pid->Pout + pid->Iout + pid->Dout;
				LimitMax(pid->out,pid->max_out)
    }
    else if (pid->mode == PID_DELTA) //增量式
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_CHANGE_DELTA)
    {
        pid->Pout = (pid->Bas_KP + pid->Gain_KP * (1 - 1.0f / expf(pid->Cp * Fabs(pid->error[0]))))*(pid->error[0] - pid->error[1]);
        pid->Iout = ((1.0f / expf(pid->Ci * Fabs(pid->error[0]))) * pid->Max_I) * ((pid->error[0] + pid->error[1])/2.0f);
        pid->out += pid->Pout + pid->Iout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ??????
  * @param[out]     pid: PID?????????
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


uint16_t BetaGeneration(float error, float epsilon)
{
    uint16_t beta = 0;
    if (Fabs(error) <= epsilon) {
        beta = 1;
    }
    return beta;
}

//--------------------------------------------------------------
//  @brief        均值滤波
//  @param        float inData       输入数据
//                float a            滤波系数
//  @return       outData            输出数据
//  @note         float型
//--------------------------------------------------------------
float data_filtering(float *filter, const float filter_data, const uint8_t filter_depth)
{
    float filter_sum = 0;
    uint8_t i;
    // 更新数据
    filter[filter_depth] = filter_data;

    for (i = 0; i < filter_depth; i++) {
        filter[i] = filter[i + 1]; // 数据左移，扔掉末尾数据
        filter_sum += filter[i];
    }
    return ((float)filter_sum / (float)filter_depth);
}

//--------------------------------------------------------------
//  @brief        一阶低通滤波
//  @param        float inData       输入数据
//                float a            滤波系数(a* 当前数据 + (1-a)*上次数据)
//  @return       outData            输出数据
//  @note         float型
//--------------------------------------------------------------
float errorfilter(float inData, float a)
{
    float outData;
    static float prevData1R = 0;
    outData                 = (1 - a) * prevData1R + a * inData;
    prevData1R              = inData;
    return outData;
}


// 高通滤波器,采样频率 100HZ,截止频率 10HZ
float NUM[3] = {0.7045881062536, 1, 0.8912509381337};
float DEN[3] = {1, -1.274238074201, 1};
/*******************************************************************/
// Name: IIR_Filter
// Description: 低通滤波器,采样频率 100Hz
// Calls:
// Input: xn-当前输入
// Return: 速度滤波后的值
/*******************************************************************/
float IIR_Filter(float xn)
{
    float Data_In[3]  = {0, 0, 0};
    float Data_Out[3] = {0, 0, 0};
    Data_In[0]        = xn;
    Data_Out[0]       = -DEN[1] * Data_Out[1] - DEN[2] * Data_Out[2] + NUM[0] * Data_In[0] + NUM[1] * Data_In[1] + NUM[2] * Data_In[2];
    Data_Out[2]       = Data_Out[1];
    Data_Out[1]       = Data_Out[0];
    Data_In[2]        = Data_In[1];
    Data_In[1]        = Data_In[0];
    return Data_Out[0];
}

