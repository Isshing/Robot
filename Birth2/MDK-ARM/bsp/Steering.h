#ifndef __STEERING_H__
#define __STEERING_H__
#include "struct_typedef.h"
typedef struct // 舵机结构体参数
{

    float KP;
    float KD;

    float P_out;
    float D_out;
    float Gyro_out;

    float Kp_Gain;
    float Base;

    float Kd_Gain;

    float result;

} Duoji_Para;

extern Duoji_Para Serve;

extern float kp_m;
extern float kd_m;
extern float kp_m_save;
extern float kd_m_save;
extern float kp_bas_save;
extern float fuzzy_P_out;
extern float fuzzy_D_out;
extern float fuzzy_kp;
extern float fuzzy_kd;
extern float fuzzy_kp_bas;

extern float els;
extern float elr;
extern float DFF[7];
extern float EFF[7];

float KP_Fuzzy(float E, float EC);
float pid_more(float error);
float pid_more_y(float error);

float pid_more_w(float error);
#endif
