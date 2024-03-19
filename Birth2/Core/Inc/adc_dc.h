#ifndef __ADC_AD_H__
#define __ADC_AD_H__


#define float32 float

#define N1    3   // 中值滤波采样点数
#define N2    8   // 平均滤波采样点数
#define ORDER 1   // 一阶低通滤波器阶数
#define ALPHA 0.8 // 一阶低通滤波器系数



extern float32 AD_1;
extern float32 AD_7;
extern float32 AD_3;
extern float32 AD_4;

extern float32 F_AD_1;
extern float32 F_AD_7;
extern float32 F_AD_3;
extern float32 F_AD_4;

void AD_main_Filter(unsigned long ,unsigned long);
float AD_median_filter(float *data, int size);
float AD_filter(float inData);

#endif
