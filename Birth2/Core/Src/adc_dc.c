#include "adc_dc.h"
#include <stdlib.h>

float32 F_AD_1 = 0;
float32 F_AD_7 = 0;
float32 F_AD_3 = 0;
float32 F_AD_4 = 0;

float32 Magnet_Err  = 0;
float32 Magnet_Err1 = 0;
float32 Magnet_Err2 = 0;
float32 Magnet_dxd  = 0;



 float AD_median_filter(float *data, int n)
{
    // 复制数据
    float *temp = (float *)malloc(n * sizeof(float));
    for (int i = 0; i < n; i++) {
        temp[i] = data[i];
    }

    // 排序
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (temp[j] < temp[i]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    // 计算中值
    float median = 0;
    if (n % 2 == 0) {
        median = (temp[n / 2 - 1] + temp[n / 2]) / 2;
    } else {
        median = temp[n / 2];
    }

    // 释放内存
    free(temp);

    return median;
}

float data1_1[N1]; // 保存中值滤波器的输入数据
float data2_1[N2]; // 保存平均滤波器的输入数据
int count1_1 = 0;  // 中值滤波器输入数据的计数器
int count2_1 = 0;  // 平均滤波器输入数据的计数器

float data1_2[N1]; // 保存中值滤波器的输入数据
float data2_2[N2]; // 保存平均滤波器的输入数据
int count1_2 = 0;  // 中值滤波器输入数据的计数器
int count2_2 = 0;  // 平均滤波器输入数据的计数器

float data1_3[N1]; // 保存中值滤波器的输入数据
float data2_3[N2]; // 保存平均滤波器的输入数据
int count1_3 = 0;  // 中值滤波器输入数据的计数器
int count2_3 = 0;  // 平均滤波器输入数据的计数器

float data1_4[N1]; // 保存中值滤波器的输入数据
float data2_4[N2]; // 保存平均滤波器的输入数据
int count1_4 = 0;  // 中值滤波器输入数据的计数器
int count2_4 = 0;  // 平均滤波器输入数据的计数器

//float AD_filter_1(float inData)
//{

//    float output = 0; // 输出值

//    // 将输入数据存储在data1数组中，用于下一次滤波
//    for (int i = N1 - 1; i > 0; i--) {
//        data1_1[i] = data1_1[i - 1];
//    }
//    data1_1[0] = inData;
//    // ips200_show_float(100, 20, inData, 4, 4);

//    // 中值滤波器
//    if (count1_1 < N1) {
//        count1_1++; // 如果 data1_1 数组中存储的数据点数不足 N1 个，则不进行中值滤波
//        output = inData;
//    } else {
//        output = AD_median_filter(data1_1, N1); // 调用中值滤波器，将滤波后的结果存储在 output 变量中
//    }

//    // ips200_show_float(100, 40, output, 4, 4);

//    // 中位值平均滤波器
//    if (count2_1 < N2 - 1) {
//        count2_1++;

//        data2_1[count2_1] = output;

//    } else {
//        // 找到最大值和最小值

//        for (int i = N2 - 1; i > 0; i--) {
//            data2_1[i] = data2_1[i - 1];
//        }
//        data2_1[0] = output;

//        float max_value = data2_1[0];
//        float min_value = data2_1[0];
//        int m           = 0;
//        int n           = 0;
//        for (int i = 1; i < N2 - 1; i++) {
//            if (data2_1[i] > max_value) {
//                m = i;
//            }
//            if (data2_1[i] < min_value) {
//                n = i;
//            }
//        }

//        // ips200_show_int(140, 140, m, 3);
//        // ips200_show_int(140, 160, n, 3);

//        // 计算剩余数据的平均值
//        float sum = 0;
//        int num   = 0;
//        for (int i = 0; i < N2 - 1; i++) {
//            // 排除最大值和最小值
//            if (i != m && i != n) {
//                sum += data2_1[i];
//                num++;
//            }
//        }

//        // 计算平均值
//        // if (m == n) {
//        output = sum / num;
//        // } else {
//        //     output = sum / (N2 - 2);
//        // }
//    }
//    // ips200_show_float(100, 60, output, 4, 4);
//    // ips200_show_float(0, 60, count2_1, 4, 4);

//    // 一阶低通滤波器
//    static float prev_output = 0;
//    output                   = ALPHA * output + (1 - ALPHA) * prev_output;
//    prev_output              = output;
//    // ips200_show_float(100, 80, output, 4, 4);
//    return output;
//}
//float AD_filter_2(float inData)
//{

//    float output = 0; // 输出值

//    // 将输入数据存储在data1数组中，用于下一次滤波
//    for (int i = N1 - 1; i > 0; i--) {
//        data1_2[i] = data1_2[i - 1];
//    }
//    data1_2[0] = inData;
//    // ips200_show_float(100, 20, inData, 4, 4);

//    // 中值滤波器
//    if (count1_2 < N1) {
//        count1_2++; // 如果 data1_2 数组中存储的数据点数不足 N1 个，则不进行中值滤波
//        output = inData;
//    } else {
//        output = AD_median_filter(data1_2, N1); // 调用中值滤波器，将滤波后的结果存储在 output 变量中
//    }

//    // ips200_show_float(100, 40, output, 4, 4);

//    // 中位值平均滤波器
//    if (count2_2 < N2 - 1) {
//        count2_2++;

//        data2_2[count2_2] = output;

//    } else {
//        // 找到最大值和最小值

//        for (int i = N2 - 1; i > 0; i--) {
//            data2_2[i] = data2_2[i - 1];
//        }
//        data2_2[0] = output;

//        float max_value = data2_2[0];
//        float min_value = data2_2[0];
//        int m           = 0;
//        int n           = 0;
//        for (int i = 1; i < N2 - 1; i++) {
//            if (data2_2[i] > max_value) {
//                m = i;
//            }
//            if (data2_2[i] < min_value) {
//                n = i;
//            }
//        }

//        // ips200_show_int(140, 140, m, 3);
//        // ips200_show_int(140, 160, n, 3);

//        // 计算剩余数据的平均值
//        float sum = 0;
//        int num   = 0;
//        for (int i = 0; i < N2 - 1; i++) {
//            // 排除最大值和最小值
//            if (i != m && i != n) {
//                sum += data2_2[i];
//                num++;
//            }
//        }

//        // 计算平均值
//        // if (m == n) {
//        output = sum / num;
//        // } else {
//        //     output = sum / (N2 - 2);
//        // }
//    }
//    // ips200_show_float(100, 60, output, 4, 4);
//    // ips200_show_float(0, 60, count2_2, 4, 4);

//    // 一阶低通滤波器
//    static float prev_output = 0;
//    output                   = ALPHA * output + (1 - ALPHA) * prev_output;
//    prev_output              = output;
//    // ips200_show_float(100, 80, output, 4, 4);
//    return output;
//}
//float AD_filter_3(float inData)
//{

//    float output = 0; // 输出值

//    // 将输入数据存储在data1数组中，用于下一次滤波
//    for (int i = N1 - 1; i > 0; i--) {
//        data1_3[i] = data1_3[i - 1];
//    }
//    data1_3[0] = inData;
//    // ips200_show_float(100, 20, inData, 4, 4);

//    // 中值滤波器
//    if (count1_3 < N1) {
//        count1_3++; // 如果 data1_3 数组中存储的数据点数不足 N1 个，则不进行中值滤波
//        output = inData;
//    } else {
//        output = AD_median_filter(data1_3, N1); // 调用中值滤波器，将滤波后的结果存储在 output 变量中
//    }

//    // ips200_show_float(100, 40, output, 4, 4);

//    // 中位值平均滤波器
//    if (count2_3 < N2 - 1) {
//        count2_3++;

//        data2_3[count2_3] = output;

//    } else {
//        // 找到最大值和最小值

//        for (int i = N2 - 1; i > 0; i--) {
//            data2_3[i] = data2_3[i - 1];
//        }
//        data2_3[0] = output;

//        float max_value = data2_3[0];
//        float min_value = data2_3[0];
//        int m           = 0;
//        int n           = 0;
//        for (int i = 1; i < N2 - 1; i++) {
//            if (data2_3[i] > max_value) {
//                m = i;
//            }
//            if (data2_3[i] < min_value) {
//                n = i;
//            }
//        }

//        // ips200_show_int(140, 140, m, 3);
//        // ips200_show_int(140, 160, n, 3);

//        // 计算剩余数据的平均值
//        float sum = 0;
//        int num   = 0;
//        for (int i = 0; i < N2 - 1; i++) {
//            // 排除最大值和最小值
//            if (i != m && i != n) {
//                sum += data2_3[i];
//                num++;
//            }
//        }

//        // 计算平均值
//        // if (m == n) {
//        output = sum / num;
//        // } else {
//        //     output = sum / (N2 - 2);
//        // }
//    }
//    // ips200_show_float(100, 60, output, 4, 4);
//    // ips200_show_float(0, 60, count2_3, 4, 4);

//    // 一阶低通滤波器
//    static float prev_output = 0;
//    output                   = ALPHA * output + (1 - ALPHA) * prev_output;
//    prev_output              = output;
//    // ips200_show_float(100, 80, output, 4, 4);
//    return output;
//}
// float AD_filter_4(float inData)
//{

//    float output = 0; // 输出值

//    // 将输入数据存储在data1数组中，用于下一次滤波
//    for (int i = N1 - 1; i > 0; i--) {
//        data1_4[i] = data1_4[i - 1];
//    }
//    data1_4[0] = inData;
//    // ips200_show_float(100, 20, inData, 4, 4);

//    // 中值滤波器
//    if (count1_4 < N1) {
//        count1_4++; // 如果 data1_4 数组中存储的数据点数不足 N1 个，则不进行中值滤波
//        output = inData;
//    } else {
//        output = AD_median_filter(data1_4, N1); // 调用中值滤波器，将滤波后的结果存储在 output 变量中
//    }

//    // ips200_show_float(100, 40, output, 4, 4);

//    // 中位值平均滤波器
//    if (count2_4 < N2 - 1) {
//        count2_4++;

//        data2_4[count2_4] = output;

//    } else {
//        // 找到最大值和最小值

//        for (int i = N2 - 1; i > 0; i--) {
//            data2_4[i] = data2_4[i - 1];
//        }
//        data2_4[0] = output;

//        float max_value = data2_4[0];
//        float min_value = data2_4[0];
//        int m           = 0;
//        int n           = 0;
//        for (int i = 1; i < N2 - 1; i++) {
//            if (data2_4[i] > max_value) {
//                m = i;
//            }
//            if (data2_4[i] < min_value) {
//                n = i;
//            }
//        }

//        // ips200_show_int(140, 140, m, 3);
//        // ips200_show_int(140, 160, n, 3);

//        // 计算剩余数据的平均值
//        float sum = 0;
//        int num   = 0;
//        for (int i = 0; i < N2 - 1; i++) {
//            // 排除最大值和最小值
//            if (i != m && i != n) {
//                sum += data2_4[i];
//                num++;
//            }
//        }

//        // 计算平均值
//        // if (m == n) {
//        output = sum / num;
//        // } else {
//        //     output = sum / (N2 - 2);
//        // }
//    }
//    // ips200_show_float(100, 60, output, 4, 4);
//    // ips200_show_float(0, 60, count2_4, 4, 4);

//    // 一阶低通滤波器
//    static float prev_output = 0;
//    output                   = ALPHA * output + (1 - ALPHA) * prev_output;
//    prev_output              = output;
//    // ips200_show_float(100, 80, output, 4, 4);
//    return output;
//}

//void AD_main_Filter(unsigned long AD_1,unsigned long AD_2)
//{
//    F_AD_1 = AD_filter_1(AD_1);
//    F_AD_7 = AD_filter_2(AD_2);
//    //F_AD_3 = AD_filter_3(AD_3);
//    //F_AD_4 = AD_filter_4(AD_4);
//}

