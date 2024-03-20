#include "QMath.h"
#include "math.h"

/*开平方函数*/
float FSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i       = *(int *)&x; // evil floating point bit level hacking
    // i = 0x5f3759df - (i >> 1);  // what the fuck?
    i = 0X5F3504F3 - (i >> 1); // 精度更高
    x = *(float *)&i;
    x = x * (1.5f - (xhalf * x * x));
    return 1 / x;
}

/*指数函数*/
float FExp(float x)
{
    x = 1.0 + x / 4096;

    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    return x;
}

/*简易atan函数，判断4个方向*/
uint8_t Atan2(float y, float x)
{
    /*--------------------------------------------
     *    角度范围        |         弧度范围
     *--------------------------------------------
     * 0     ~ 22.5  ------> 0         ~ 0.3926990
     * 22.5  ~ 67.5  ------> 0.3926990 ~ 1.1780972
     * 67.5  ~ 112.5 ------> 1.1780972 ~ 1.9634954
     * 112.5 ~ 157.5 ------> 1.9634954 ~ 2.7488935
     * 157.5 ~ 180   ------> 2.7488935 ~ 3.1415926
     *--------------------------------------------
     *         y/x值对应弧度
     *  0          ----  0.41421356  水平方向
     *  0.41421347 ----  2.41421356  右上、左下
     *  2.41421326 ---- -2.41421356  竖直方向
     * -2.41421362 ---- -0.41421356  左上、右下
     * -0.41421365 ----  0           水平方向
     ********************************************/

    float f32_tanNum;
    uint8_t u8_Alpha; // 返回角度
    f32_tanNum = y / x;
    if (f32_tanNum > -0.41421365 && f32_tanNum < 0.41421356)
        u8_Alpha = 0; // 水平方向
    else if (f32_tanNum >= 0.41421356 && f32_tanNum < 2.41421356)
        u8_Alpha = 1; // 右上、左下
    else if (f32_tanNum <= -0.41421356 && f32_tanNum > -2.41421362)
        u8_Alpha = 3; // 左上、右下
    else
        u8_Alpha = 2; // 竖直方向

    return u8_Alpha; // 方向
}

float arctan(float a, float b) // 利用麦克劳林展开式计算
{
    float ans;
    float x1 = 0, x2 = 0, x;
    int i;

    if (b == 0 || a == 0)
        return 0;
    else
        x = a / b;

    if (x > 1 || x < -1)
        x1 = 1 / x;
    else
        x1 = x;
    ans = x1;

    x2 = x1;
    for (i = 1; i < 20; i++) {
        x1 = x1 * x2 * x2 * (-1) * (2 * i - 1) / (2 * i + 1);
        if (abs(x1) < 0.000000001)
            break;
        ans += x1;
    }
    if (x < -1)
        return (-3.141592566 / 2 - ans);
    else if (x > 1)
        return (3.141592566 / 2 - ans);
    else
        return ans;
}

/*快速排序*/
void Quicksort(float array[], uint8_t maxlen, uint8_t begin, uint8_t end)
{
    uint8_t i, j;

    if (begin < end) {
        i = begin + 1; // 将array[begin]作为基准数，因此从array[begin+1]开始与基准数比较！
        j = end;       // array[end]是数组的最后一位

        while (i < j) {
            if (array[i] > array[begin]) // 如果比较的数组元素大于基准数，则交换位置。
            {
                Swap(&array[i], &array[j]); // 交换两个数
                j--;
            } else
                i++; // 将数组向后移一位，继续与基准数比较。
        }

        if (array[i] >= array[begin]) // 这里必须要取等“>=”，否则数组元素由相同的值时，会出现错误！
            i--;

        Swap(&array[begin], &array[i]); // 交换array[i]与array[begin]

        Quicksort(array, maxlen, begin, i);
        Quicksort(array, maxlen, j, end);
    }
}

/*交换数值*/
void Swap(float *a, float *b)
{
    float temp;

    temp = *a;
    *a   = *b;
    *b   = temp;
}



/*指数计算*/
float QPow(float base, int8_t exp)
{
    float f32_ans = 1;
    while (exp) {
        if (exp & 1)
            f32_ans *= base;
        base *= base;
        exp >>= 1;
    }
    return f32_ans;
}

/*绝对值函数*/
uint16_t Kabs(int16_t num)
{
    if (num >= 0)
        return num;
    return -num;
}

/*浮点数绝对值函数*/
float Fabs(float num)
{
    if (num >= 0)
        return num;
    return -num;
}

/*范围限制函数*/
int16_t clip(int16_t x, int16_t low, int16_t high)
{
    return x > high ? high : x < low ? low
                                     : x;
}

/*浮点范围限制*/
float fclip(float x, float low, float high)
{
    return x > high ? high : x < low ? low
                                     : x;
}

/*遍历寻找数组最大值*/
float fFindABSMax(float *f, int16_t len)
{
    float f32_Max   = 0;
    int16_t int16_t_loopi = -1;

    while (++int16_t_loopi < len) {
        if (fabs(*(f + int16_t_loopi)) > fabs(f32_Max)) {
            f32_Max = *(f + int16_t_loopi);
        }
    }

    return f32_Max;
}

/*三点计算曲率*/
// 这个曲率计算的函数有点bug
// float32 ThreePointsCurvature(INT_POINT_INFO t_P1, INT_POINT_INFO t_P2, INT_POINT_INFO t_P3)
//{
//     float32 f32_K;
//     int16_t f32_AB = (t_P2.m_i16x - t_P1.m_i16x) * (t_P2.m_i16x - t_P1.m_i16x) + (t_P2.m_i16y - t_P1.m_i16y) * (t_P2.m_i16y - t_P1.m_i16y);
//     int16_t f32_AC = (t_P3.m_i16x - t_P1.m_i16x) * (t_P3.m_i16x - t_P1.m_i16x) + (t_P3.m_i16y - t_P1.m_i16y) * (t_P3.m_i16y - t_P1.m_i16y);
//     int16_t f32_BC = (t_P2.m_i16x - t_P3.m_i16x) * (t_P2.m_i16x - t_P3.m_i16x) + (t_P2.m_i16y - t_P3.m_i16y) * (t_P2.m_i16y - t_P3.m_i16y);
//
//     /*弯道向右拐面积为负,否则为正*/
//     int16_t f32_S = ((t_P2.m_i16x - t_P1.m_i16x) * (t_P3.m_i16y - t_P1.m_i16y) - (t_P3.m_i16x - t_P1.m_i16x) * (t_P2.m_i16y - t_P1.m_i16y));
//
//
//     if(f32_AB * f32_AC * f32_BC == 0) f32_K = 0;
//     else f32_K = 2 * f32_S /FSqrt( (float32) (f32_AB * f32_AC * f32_BC) );
//
//
//     return f32_K;
// }

float process_curvity(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3)
{
    float K;
    int32_t S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    // 面积的符号表示方向
    int32_t q1 = (int16_t)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    int32_t AB = (int32_t)sqrt((double)q1);
    q1       = (int32_t)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    int32_t BC = (int32_t)sqrt((double)q1);
    q1       = (int32_t)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    int32_t AC = (int32_t)sqrt((double)q1);
    if (AB * BC * AC == 0) {
        K = 0;
    } else
        K = (double)4 * S_of_ABC / (AB * BC * AC);
    return K;
}

// 这里之前使用float似乎会溢出
double Fprocess_curvity(float x1, float y1, float x2, float y2, float x3, float y3)
{
    double K;
    double S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    // 面积的符号表示方向
    double q1 = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double AB = sqrt(q1);
    q1         = ((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    double BC = sqrt(q1);
    q1         = ((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    double AC = sqrt(q1);
    if (AB * BC * AC == 0) {
        K = 0;
    } else
        K = 4 * S_of_ABC / (AB * BC * AC);
    return K;
}

// /*线性回归，选取一段数据进行拟合*/
// void Regression(INT_POINT_INFO *Point, double *Slope, double *Sita, double *Intersect, int16_t startLine, int16_t endLine)
// {
//     // 计算线段长度
//     int16_t int16_LineNum = endLine - startLine;

//     // 初始化 int16_sumX、int16_sumY、f32_aveX、f32_aveY、f32_sumUP、f32_sumDOWN、*Slope 和 *Intersect 为 0
//     int32_t int16_sumX    = 0;
//     int32_t int16_sumY    = 0;
//     float f32_aveX    = 0;
//     float f32_aveY    = 0;
//     float f32_sumUP   = 0;
//     float f32_sumDOWN = 0;
//     *Slope              = 0;
//     *Intersect          = 0;

//     // 定义 f32_k 和 f32_b
//     float f32_k;
//     float f32_b;

//     // 如果线段长度不为 0，则进行线性回归
//     if (int16_LineNum != 0) {
//         // 计算所有点的 x 和 y 坐标之和
//         for (int16_t i = startLine; i < endLine; i++) {
//             int16_sumX += Point[i].m_i16x;
//             int16_sumY += Point[i].m_i16y;
//         }

//         // 计算 x 和 y 坐标的平均值
//         f32_aveX = (float)int16_sumX / int16_LineNum;
//         f32_aveY = (float)int16_sumY / int16_LineNum;

//         // 计算斜率和截距
//         for (int16_t i = startLine; i < endLine; i++) {
//             f32_sumUP += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16y - f32_aveY);
//             f32_sumDOWN += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16x - f32_aveX);
//         }

//         // 计算斜率
//         if (f32_sumDOWN == 0)
//             f32_k = 0;
//         else
//             f32_k = f32_sumUP / f32_sumDOWN;
//         *Sita = arctan(f32_sumUP, f32_sumDOWN);
//         // 计算截距
//         f32_b = f32_aveY - f32_aveX * f32_k;
//     }

//     // 将斜率和截距赋值给指针 Slope 和 Intersect 所指向的变量
//     *Slope     = f32_k;
//     *Intersect = f32_b;
// }

// /*线性回归，选取一段数据进行拟合*/
// void FRegression(FLOAT_POINT_INFO *Point, double *Slope, double *Intersect, int16_t startLine, int16_t endLine)
// {
//     int16_t int16_LineNum = endLine - startLine;

//     double int16_sumX = 0;
//     double int16_sumY = 0;
//     double f32_aveX   = 0;
//     double f32_aveY   = 0;

//     double f32_sumUP = 0;
//     ;
//     double f32_sumDOWN = 0;

//     *Slope        = 0;
//     *Intersect    = 0;
//     double f32_k = 0;
//     double f32_b = 0;

//     if (int16_LineNum > 0) {
//         for (int16_t i = startLine; i < endLine; i++) {
//             int16_sumX += Point[i].m_i16x;
//             int16_sumY += Point[i].m_i16y;
//         }

//         f32_aveX = (double)int16_sumX / int16_LineNum;
//         f32_aveY = (double)int16_sumY / int16_LineNum;

//         for (int16_t i = startLine; i < endLine; i++) {
//             f32_sumUP += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16y - f32_aveY);
//             f32_sumDOWN += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16x - f32_aveX);
//         }

//         /*斜率*/
//         if (f32_sumDOWN == 0)
//             f32_k = 0;
//         else
//             f32_k = f32_sumUP / f32_sumDOWN;

//         /*截距*/
//         f32_b = f32_aveY - f32_aveX * f32_k;
//     }

//     *Slope     = f32_k;
//     *Intersect = f32_b;
// }

