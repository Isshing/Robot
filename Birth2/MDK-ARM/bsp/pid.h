#ifndef __PID_H__
#define __PID_H__

#include "struct_typedef.h"
#include "Qmath.h"

/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidÊµÏÖº¯Êý£¬°üÀ¨³õÊ¼»¯£¬PID¼ÆËãº¯Êý£¬
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Íê³É
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

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

#define MOTOR1_DIR                A1                   // 定义3电机正反转引脚
#define MOTOR1_PWM                TIM5_PWM_MAP0_CH1_A0 // 定义3电机PWM引脚

#define MOTOR2_DIR                A2                   // 定义4电机正反转引脚
#define MOTOR2_PWM                TIM5_PWM_MAP0_CH4_A3 // 定义4电机PWM引脚

#define Distance_Col              0.155 // 车身的前后轮中心距

#define Distance_Row              0.118 // 两后轮中心距

#define MIN(a, b)                 (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                 (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

/*定义PID对象类型*/
typedef struct // 位置型普通PID
{
    float maximum;  /*输出值上限*/
    float minimum;  /*输出值下限*/
    float setpoint; // 设定值
    float result;   // 输出值
    float integral; // 积分值

} PID_1;

typedef struct // 增量型普通PID
{

    float maximum;  /*输出值上限*/
    float minimum;  /*输出值下限*/
    float setpoint; // 设定值

    float result;     // 输出值
    float integral;   // 积分值
    float derivative; // 微分值

    float P_out;
    float I_out;

} PID_2;

typedef struct // 后轮结构体参数
{
    // 变积分参数
    float L_Max_I;
    float L_Ci;

    float R_Max_I;
    float R_Ci;

    // 变比例参数
    float L_Bas_KP;
    float L_Gain_KP;
    float L_Cp;

    float R_Bas_KP;
    float R_Gain_KP;
    float R_Cp;

    float L_increment;
    float L_pError;
    float L_iError;
    float L_Change_P;
    float L_Change_I;

    float R_increment;
    float R_pError;
    float R_iError;
    float R_Change_P;
    float R_Change_I;

    enum { // 枚举
        MODE_NORMAL,
        MODE_BANGBANG,
        MODE_SOFT,
    } motor_mode;

} Motor_Para;

typedef struct // 差速结构体参数
{
    float K;       // 补偿系数
    float maximun; // 差速最大值
    float minimum; // 差速最小值

} Chasu_Para;

extern PID_2 Motor_Left;    // 左轮变量
extern PID_2 Motor_Right;   // 右轮变量
extern Motor_Para MOTOR;    // PID参数结构体
extern Motor_Para LORDTEST; // PID参数结构体
extern Chasu_Para CHASU;    // 差速参数结构体

extern uint8_t Speed_Bia;

void Back_Wheel_Out(int32_t R_outPWM, int32_t L_outPWM);
void Motor_L_Control(PID_2 *vPID, Motor_Para *Motor, int16_t processValue);
void Motor_R_Control(PID_2 *vPID, Motor_Para *Motor, int16_t processValue);
void Motor_L_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16_t processValue);
void Motor_R_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16_t processValue);
void Motor_L_Control_Position_Advance_differential(PID_2 *vPID, Motor_Para *Motor, int16_t processValue);

uint16_t BetaGeneration(float error, float epsilon);
float errorfilter(float inData, float a);
float data_filtering(float *filter, const float filter_data, const uint8_t filter_depth);

void Motor_L_Control_Change_Integral(float setpoint, PID_2 *vPID, Motor_Para *Motor, int16_t processValue);
void Motor_R_Control_Change_Integral(float setpoint, PID_2 *vPID, Motor_Para *Motor, int16_t processValue);



//----------------------------------------------------------我是分割线-------------------------------------------------------------//
//----------------------------------------------------------我是分割线-------------------------------------------------------------//
//----------------------------------------------------------我是分割线-------------------------------------------------------------//
//----------------------------------------------------------我是分割线-------------------------------------------------------------//
// PIDģ模板

////普通位置型PID(梯形积分)
// void PIDRegulation_1(PID_1 *vPID, float processValue)
//{
//
//     float thisError;
//     float result;
//     thisError = vPID->setpoint - processValue;
//     if (fabs(thisError) > vPID->deadband) {
//         vPID->integral += (thisError + vPID->lasterror) / 2;
//         result = vPID->proportiongain * thisError + vPID->integralgain * vPID->integral + vPID->derivativegain * (thisError - vPID->lasterror);
//
//     } else {
//         if ((abs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (abs(processValue - vPID->minimum) < vPID->deadband)) {
//             result = vPID->minimum;
//         }
//     }
//     /*对输出限幅，避免超调和积分饱和问题*/
//     if (result >= vPID->maximum) {
//         result = vPID->maximum;
//     }
//     if (result <= vPID->minimum) {
//         result = vPID->minimum;
//     }
//     vPID->lasterror = thisError;
//     vPID->result = result;
//     vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
// }

// 普通增量型PID(梯形积分)
// void PIDRegulation_2 (PID_2 *vPID, float processValue)
//{
//     float thisError;
//     float result;
//     float increment;
//     float pError, dError, iError;
//
//     thisError = vPID->setpoint - processValue; //得到偏差值
//     result = vPID->result;
//
//     if (fabs(thisError) > vPID->deadband)
//     {
//         pError = thisError - vPID->lasterror;
//         iError = (thisError + vPID->lasterror) / 2.0;
//         dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;
//
//         increment = vPID->proportiongain * pError + vPID->integralgain * iError + vPID->derivativegain * dError;
//     }
//     else
//     {
//         if ((fabs(vPID->setpoint - vPID->minimum) < vPID->deadband)
//                 && (fabs(processValue - vPID->minimum) < vPID->deadband))
//         {
//             result = vPID->minimum;
//         }
//         increment = 0.0;
//     }
//     result = result + increment;
//
//     /*对输出限值，避免超调和积分饱和问题*/
//     if (result >= vPID->maximum)
//     {
//         result = vPID->maximum;
//     }
//     if (result <= vPID->minimum)
//     {
//         result = vPID->minimum;
//     }
//
//     vPID->preerror = vPID->lasterror; //存放偏差用于下次运算
//     vPID->lasterror = thisError;
//     vPID->result = result;
//     vPID->output = ((result - vPID->minimum) / (vPID->maximum - vPID->minimum)) * 100.0;
// }

#endif
