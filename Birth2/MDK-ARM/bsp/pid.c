#include "pid.h"

//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
PID_2 Motor_Left;    // 左轮电机变量
PID_2 Motor_Right;   // 右轮电机变量
Motor_Para MOTOR;    // 电机参数结构体变量
Motor_Para LORDTEST; // 检测上载是否正常

Chasu_Para CHASU; // 差速参数结构体变量

uint8_t Speed_Bia = 3; // 左右轮速度差(保证直行时走直线)

float history[4];

//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     左轮增量式PID(变积分)
//  @param     float32 setpoint         期望速度
//             PID_1 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID,死区限制，变积分
//--------------------------------------------------------------
void Motor_L_Control_Change_Integral(float setpoint, PID_2 *vPID, Motor_Para *Motor, int16_t processValue)
{
    float thisError;
    float result;
    float increment;
    float pError = 0;
    float iError = 0;

    static float lasterror = 0; // 前一拍偏差

    thisError         = setpoint - processValue; // 得到偏差值
    Motor->L_Change_P = Motor->L_Bas_KP + Motor->L_Gain_KP * (1 - 1.0 / FExp(Motor->L_Cp * Fabs(thisError)));
    Motor->L_Change_I = (1.0 / FExp(Motor->L_Ci * Fabs(thisError))) * Motor->L_Max_I;

    result = vPID->result;

    iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围内，正常积分

    pError      = thisError - lasterror;
    vPID->P_out = Motor->L_Change_P * pError;
    vPID->I_out = Motor->L_Change_I * iError;
    if (MOTOR.motor_mode == MODE_SOFT) {
        vPID->I_out = MINMAX(vPID->I_out, -20, 20);
    }
    vPID->P_out = MINMAX(vPID->P_out, -7000, 7000);
    increment   = vPID->P_out + vPID->I_out; // 变积分不用积分分离，因为积分分离本质就是变积分的一种特殊情况
    result      = result + increment;

    lasterror    = thisError;
    vPID->result = MINMAX(result, vPID->minimum, vPID->maximum);
}

//--------------------------------------------------------------
//  @brief     右轮增量式PID(变积分)
//  @param     float32 setpoint        期望速度
//             PID_1 *vPID             速度环PID常用量
//             Motor_Para *Motor       PID参数结构体;
//             processValue            转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID，死区限制，变积分
//--------------------------------------------------------------
void Motor_R_Control_Change_Integral(float setpoint, PID_2 *vPID, Motor_Para *Motor, int16_t processValue)
{
    float thisError;
    float result;
    float increment        = 0;
    float pError           = 0;
    float iError           = 0;
    static float lasterror = 0; // 前一拍偏差

    thisError         = setpoint - processValue; // 得到偏差值
    Motor->R_Change_P = Motor->R_Bas_KP + Motor->R_Gain_KP * (1 - 1.0 / FExp(Motor->R_Cp * Fabs(thisError)));
    Motor->R_Change_I = (1.0 / FExp(Motor->R_Ci * Fabs(thisError))) * Motor->R_Max_I;
    result            = vPID->result;

    iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围内，正常积分

    pError = thisError - lasterror;

    vPID->P_out = Motor->R_Change_P * pError;
    vPID->I_out = Motor->R_Change_I * iError;
    vPID->P_out = MINMAX(vPID->P_out, -7000, 7000);
    if (MOTOR.motor_mode == MODE_SOFT) {
        vPID->I_out = MINMAX(vPID->I_out, -20, 20);
    }
    increment = vPID->P_out + vPID->I_out;
    result    = result + increment;

    lasterror    = thisError;
    vPID->result = MINMAX(result, vPID->minimum, vPID->maximum);
}
//--------------------------------------------------------------
//  @brief     左轮增量式PID
//  @param     PID_1 *vPID              速度环PID常用量;
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID，死区限制，梯形积分
//--------------------------------------------------------------
// void Motor_L_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
// {

//     float thisError;
//     float result;
//     float increment;
//     float pError, dError, iError;
//     static float lasterror = 0; // 前一拍偏差
//     static float preerror  = 0; // 前两拍偏差

//     thisError = vPID->setpoint - processValue; // 得到偏差值

//     result = vPID->result;

//     if (vPID->result > vPID->maximum - 500) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
//         if (thisError <= 0) {
//             iError = (thisError + lasterror) / 2.0;
//         }
//     } else if (vPID->result < vPID->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
//         if (thisError >= 0) {
//             iError = (thisError + lasterror) / 2.0;
//         }
//     } else {
//         iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
//     }
//     pError = thisError - lasterror;
//     dError = thisError - 2 * lasterror + preerror;

//     if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
//     {
//         increment = Motor->L_P * pError + Motor->L_I * iError + Motor->L_D * dError;
//     } else // 如果偏差大，取消积分作用
//     {
//         increment = Motor->L_P * pError + Motor->L_D * dError;
//     }

//     result = result + increment;

//     /*对输出限幅，避免超调和积分饱和问题*/
//     result = result >= vPID->maximum ? vPID->maximum : result;
//     result = result <= vPID->minimum ? vPID->minimum : result;

//     preerror     = lasterror; // 存放偏差用于下次运算
//     lasterror    = thisError;
//     vPID->result = result;
// }

//--------------------------------------------------------------
//  @brief     右轮增量式PID
//  @param     PID_2 *vPID              速度环PID常用量;
//             Motor_Para *Motor        PID参数结构体;
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PI，死区限制，梯形积分
//--------------------------------------------------------------
// void Motor_R_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
// {

//     float thisError;
//     float result;
//     float increment;
//     float pError, dError, iError;
//     static float lasterror = 0; // 前一拍偏差
//     static float preerror  = 0; // 前两拍偏差

//     thisError = vPID->setpoint - processValue; // 得到偏差值

//     result = vPID->result;

//     if (vPID->result > vPID->maximum - 500) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
//         if (thisError <= 0) {
//             iError = (thisError + lasterror) / 2.0;
//         }
//     } else if (vPID->result < vPID->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
//         if (thisError >= 0) {
//             iError = (thisError + lasterror) / 2.0;
//         }
//     } else {
//         iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
//     }
//     pError = thisError - lasterror;
//     dError = thisError - 2 * lasterror + preerror;

//     if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
//     {
//         increment = Motor->R_P * pError + Motor->R_I * iError + Motor->R_D * dError;
//     } else // 如果偏差大，取消积分作用
//     {
//         increment = Motor->R_P * pError + Motor->R_D * dError;
//     }

//     result = result + increment;

//     /*对输出限幅，避免超调和积分饱和问题*/
//     result = result >= vPID->maximum ? vPID->maximum : result;
//     result = result <= vPID->minimum ? vPID->minimum : result;

//     preerror     = lasterror; // 存放偏差，用于下次运算
//     lasterror    = thisError;
//     vPID->result = result;
// }

//--------------------------------------------------------------
//  @brief     左轮位置式PID
//  @param     PID_2 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      位置式PI，死区限制
//--------------------------------------------------------------
// void Motor_L_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
//{
//
//    float thisError;
//    float result;
//    static float lasterror = 0; // 前一拍偏差
//
//    thisError = vPID->setpoint - processValue;
//    result    = vPID->result;
//
//    vPID->integral += (thisError + lasterror) / 2;
//    result = Motor->L_P * thisError + Motor->L_I * vPID->integral + Motor->L_D * (thisError - lasterror);
//
//    lasterror    = thisError;
//    vPID->result = MINMAX(result, vPID->minimum, vPID->maximum);
//}

//--------------------------------------------------------------
//  @brief     右轮位置式PID
//  @param     PID_2 *vPID                   速度环常用量
//             Motor_Para *Motor             PID参数结构体
//             processValue                  转速(编码器值)
//  @return    void        没求得
//  @note      位置式，死区限制
//--------------------------------------------------------------
// void Motor_R_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
//{
//
//    float thisError;
//    float result;
//    static float lasterror = 0; // 前一拍偏差
//    thisError              = vPID->setpoint - processValue;
//    result                 = vPID->result;
//
//    vPID->integral += (thisError + lasterror) / 2;
//    result = Motor->R_P * thisError + Motor->R_I * vPID->integral + Motor->R_D * (thisError - lasterror);
//    lasterror    = thisError;
//    vPID->result = MINMAX(result, vPID->minimum, vPID->maximum);
//}

//--------------------------------------------------------------
//  @brief       积分分离因子判断函数
//  @param        float error            差值
//                float epsilon        界限
//  @return       beta
//  @note
//--------------------------------------------------------------
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

//--------------------------------------------------------------
//  @brief        滑动平均滤波(原始数据)
//  @param        float value        输入值
//                uint8 time         取平均个数
//  @return       sum / K            输出数据
//  @note         float型
//--------------------------------------------------------------
float Filter_ave_DuojiData(float value, uint8_t time)
{
    int i, j;
    float sum = 0;

    int factor[time];
    int K                   = 0;
    static int filter_index = 0;
    static int buff_init    = 0;

    for (i = 0; i < time; i++) {
        factor[i] = i + 1;
        K += i + 1;
    }
    if (buff_init == 0) {
        history[filter_index] = value;
        filter_index++;

        if (filter_index >= time - 1) {

            buff_init = 1;
        }

    } else {
        history[filter_index] = value;
        filter_index++;
        if (filter_index >= time - 1) {
            filter_index = 0;
        }
        j = filter_index;
        for (i = 0; i <= time - 1; i++) {
            sum += history[j] * factor[i];
            j++;
            if (j == time - 1) {
                j = 0;
            }
        }
    }
    return sum / K;
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

// #define numStages  4                /* IIR滤波的阶数 */

// /* 巴特沃斯带通滤波器系数0.16Hz 0.66Hz*/
// const float a[numStages+1] = {1.0f, -3.7589173f, 5.32081326298f, -3.362601167f, 0.8008026466657f};
// const float b[numStages+1] = {0.00554271721f, 0.0f, -0.0110854344f, 0.0f, 0.00554271721f};

// static float old_x[numStages] = {0.0f};
// static float old_y[numStages] = {0.0f};
// static int pos = 0;

// float arm_iir_f32_bp(float input)
// {
//     int i,p;
//     float output = b[0] * input;
//     for(i=1; i<=numStages; i++)
//     {
//         p=(pos + numStages - i)%numStages;
//         output += b[i] * old_x[p] - a[i] * old_y[p];
//     }
//     if(numStages > 1) {
//         old_x[pos] = input;
//         old_y[pos] = output;
//         pos = (pos+1)%numStages;
//     }

//     return output;
// }

// /*
//  * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
//  * Generated by MATLAB(R) 9.0 and the Signal Processing Toolbox 7.2.
//  * Generated on: 23-Apr-2023 04:45:47
//  */

// /*
//  * Discrete-Time IIR Filter (real)
//  * -------------------------------
//  * Filter Structure    : Direct-Form II, Second-Order Sections
//  * Number of Sections  : 1
//  * Stable              : Yes
//  * Linear Phase        : No
//  */

// /* General type conversion for MATLAB generated C-code  */
// #include "tmwtypes.h"
// /*
//  * Expected path to tmwtypes.h
//  * E:\Matlab\extern\include\tmwtypes.h
//  */
// #define MWSPT_NSEC 3
// const int NL[MWSPT_NSEC] = { 1,3,1 };
// const real64_T NUM[MWSPT_NSEC][3] = {
//   {
//      0.7045881062536,                 0,                 0
//   },
//   {
//                    1,                -2,                 1
//   },
//   {
//      0.8912509381337,                 0,                 0
//   }
// };
// const int DL[MWSPT_NSEC] = { 1,3,1 };
// const real64_T DEN[MWSPT_NSEC][3] = {
//   {
//                    1,                 0,                 0
//   },
//   {
//                    1,   -1.274238074201,    0.544114350813
//   },
//   {
//                    1,                 0,                 0
//   }
// };
