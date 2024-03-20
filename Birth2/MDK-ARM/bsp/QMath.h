#ifndef _QMATH_H_
#define _QMATH_H_
#include "math.h"
#include "struct_typedef.h"

/*开平方*/
float FSqrt(float x);

/*指数计算*/
float FExp(float x);

/*反正切，用于梯度方向计算*/
uint8_t Atan2(float y, float x);

float arctan(float a, float b);

/*快速排序*/
void Quicksort(float array[], uint8_t maxlen, uint8_t begin, uint8_t end);

/*数值交换*/
void Swap(float *a, float *b);

/*指数计算*/
float QPow(float base, int8_t exp);

/*绝对值*/
uint16_t Kabs(int16_t num);

/*浮点数绝对值函数*/
float Fabs(float num);

/*范围限制*/
int16_t clip(int16_t x, int16_t low, int16_t high);

/*浮点范围限制*/
float fclip(float x, float low, float high);

/*遍历寻找数组最大值*/
float fFindABSMax(float *f, int16_t len);

/*三点计算曲率*/
// float32 ThreePointsCurvature(INT_POINT_INFO t_P1, INT_POINT_INFO t_P2, INT_POINT_INFO t_P3);
// float process_curvity(int16 x1, int16 y1, int16 x2, int16 y2, int16 x3, int16 y3);
// double Fprocess_curvity(float32 x1, float32 y1, float32 x2, float32 y2, float32 x3, float32 y3);

/*线性回归*/
// void Regression(INT_POINT_INFO *Point, float64 *Slope, float64 *Sita, float64 *Intersect, int16 startLine, int16 endLine);
// void FRegression(FLOAT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine);

// /*赛道方差绝对值计算*/
// void Variance(INT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine);
// void FVariance(FLOAT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine);

#endif /* CODE_QMATH_H_ */
