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
    float abs_error = fabs(pid->error[0]);

    //判断死区
//    if (abs_error < 50)
//    {
//        pid->error[0] = 0.0f;
//        abs_error = 0.0f;
//    }

    if (pid->mode == PID_POSITION) //位置式
    {
				
        pid->Pout = pid->Kp * pid->error[0];
//				if(fabs(pid->error[0])<250){
//					pid->Iout += pid->Ki * pid->error[0];
//				}else{
//					pid->Iout = 0;
//				}
				pid->Dout = pid->Kd * (pid->error[0] - pid->error[1]);
				LimitMax(pid->Iout,pid->max_iout);
				pid->out = pid->Pout + pid->Iout + pid->Dout;
				LimitMax(pid->out,pid->max_out);
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





MovingAverageFilter_t filters[8];  // 为四个电机定义四个滤波器

// 初始化滤波器
void initFilters(void) {
    for (int i = 0; i < 8; i++) {
        filters[i].index = 0;
        for (int j = 0; j < SAMPLE_SIZE; j++) {
            filters[i].samples[j] = 0.0f;
        }
    }
}

// 添加新样本并计算移动平均值
float movingAverageFilter(int motor_index, float new_sample) {
    MovingAverageFilter_t *filter = &filters[motor_index];
    filter->samples[filter->index++] = new_sample;
    if (filter->index >= SAMPLE_SIZE) {
        filter->index = 0;
    }

    float sum = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        sum += filter->samples[i];
    }
    return sum / SAMPLE_SIZE;
}

