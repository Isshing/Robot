#ifndef __PID_H__
#define __PID_H__

#include "struct_typedef.h"
#include "Qmath.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA,
    PID_CHANGE_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
    fp32 Bas_KP;
    fp32 Gain_KP;
    fp32 Cp;
    fp32 Ci;
    fp32 Max_I;
    
    fp32 max_out;  
    fp32 max_iout; 

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  
    fp32 error[3]; 

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */

extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */

extern void PID_clear(pid_type_def *pid);

#define MIN(a, b)                 (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                 (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)


#endif
