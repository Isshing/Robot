#include "Steering.h"
#include <math.h>
#include "pid.h"

float kp_m        = 4;
float kd_m        = 1.5;
float kp_m_save   = 0;
float kd_m_save   = 0;
float kp_bas_save = 0;

float fuzzy_P_out  = 0;
float fuzzy_D_out  = 0;
float fuzzy_kp     = 0;
float fuzzy_kd     = 0;
float fuzzy_ki     = 0.004;
float fuzzy_I_out  = 0;
float fuzzy_kp_bas = 0.3;

float DFF[7] = {-33, -22, -11, 0, 11, 22, 33};
float EFF[7] = {-9, -6, -3, 0, 3, 6, 9};




float w_DFF[7] = {-66, -44, -22, 0, 22, 44, 66};
float w_EFF[7] = {-9, -6, -3, 0, 3, 6, 9};


float kp_w        = 1.45;
float kd_w        = 1.05;
float kp_w_save   = 0;
float kd_w_save   = 0;
float kp_bas_save_w = 0;

float w_P_out  = 0;
float w_D_out  = 0;
float w_kp     = 0;
float w_kd     = 0;
float w_ki     = 0.001;
float w_I_out  = 0;
float w_kp_bas = 0.3;



float y_DFF[7] = {-33, -22, -11, 0, 11, 22, 33};
float y_EFF[7] = {-9, -6, -3, 0, 3, 6, 9};


float kp_y        = 3;
float kd_y        = 1.5;
float kp_y_save   = 0;
float kd_y_save   = 0;
float kp_bas_save_y = 0;

float y_P_out  = 0;
float y_D_out  = 0;
float y_kp     = 0;
float y_kd     = 0;
float y_ki     = 0.002;
float y_I_out  = 0;
float y_kp_bas = 0.3;

float gaussmf(float x, float u, float a)
{
    return 0.39894228 / a * exp(-(x - u) * (x - u) / (2 * a * a));
}

float gaussmf_ec(float x, float u)
{
    return exp(-(x - u) * (x - u) * 2);
}

float cuf_w(float x, float u)
{
    if (x >= u && x <= u + 2) {
        return -0.5 * (x - u) + 1;
    } else if (x < u && x >= u - 2) {
        return 0.5 * (x - u) + 1;
    } else {
        return 0;
    }
}

float uf(float x, float a, float b, float c)
{
    if (x <= a)
        return 0;
    else if ((a < x) && (x <= b))
        return (x - a) / (b - a);
    else if ((b < x) && (x <= c))
        return (c - x) / (c - b);
    else if (x > c)
        return 0;
    return 0;
}

float cuf(float x, float a, float b, float c)
{
    float y, z;
    z = (b - a) * x + a;
    y = c - (c - b) * x;
    return (y + z) / 2;
}

float ufl(float x, float a, float b)
{
    if (x <= a)
        return 1;
    else if ((a < x) && (x <= b))
        return (b - x) / (b - a);
    else if (x > b)
        return 0;
    return 0;
}

float cufl(float x, float a, float b)
{
    return (b - (b - a) * x);
}

float ufr(float x, float a, float b)
{
    if (x <= a)
        return 0;
    if ((a < x) && (x < b))
        return (x - a) / (b - a);
    if (x >= b)
        return 1;
    return 0;
}

float cufr(float x, float a, float b)
{
    return (b - a) * x + a;
}

float fand(float a, float b)
{
    return (a < b) ? b : a;
}

float Abs(float a)
{
    if (a > 0)
        return a;
    else
        return (0 - a);
}


 float KP_Fuzzy(float E, float EC)
{

    int rule_p[7][7] =
        {
            {6, 5, 4, 4, 3, 0, 0}, //-36
            {6, 4, 3, 3, 2, 0, 0}, //-24
            {4, 3, 2, 1, 0, 1, 2}, //-12
            {2, 1, 1, 0, 1, 1, 2}, // 0
            {2, 1, 0, 1, 2, 3, 4}, // 12
            {0, 0, 2, 3, 3, 4, 6}, // 24
            {0, 1, 3, 4, 4, 5, 6}, // 36
        };

    uint8 i2;


    float UFF[7];

    for (i2 = 0; i2 < 7; i2++) {
        UFF[i2] = kp_m / 6 * i2;
    }

    float U     = 0;
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};

    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;

    if (E > EFF[0] && E < EFF[6]) {
        if (E <= EFF[1]) {
            Pn    = -2;
            PF[0] = (EFF[1] - E) / (EFF[1] - EFF[0]);
        } else if (E <= EFF[2]) {
            Pn    = -1;
            PF[0] = (EFF[2] - E) / (EFF[2] - EFF[1]);
        } else if (E <= EFF[3]) {
            Pn    = 0;
            PF[0] = (EFF[3] - E) / (EFF[3] - EFF[2]);
        } else if (E <= EFF[4]) {
            Pn    = 1;
            PF[0] = (EFF[4] - E) / (EFF[4] - EFF[3]);
        } else if (E <= EFF[5]) {
            Pn    = 2;
            PF[0] = (EFF[5] - E) / (EFF[5] - EFF[4]);
        } else if (E <= EFF[6]) {
            Pn    = 3;
            PF[0] = (EFF[6] - E) / (EFF[6] - EFF[5]);
        }
    }

    else if (E <= EFF[0]) {
        Pn    = -2; /*  ??? */
        PF[0] = 1;
    } else if (E >= EFF[6]) {
        Pn    = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    if (EC > DFF[0] && EC < DFF[6]) {
        if (EC <= DFF[1]) {
            Dn    = -2;
            DF[0] = (DFF[1] - EC) / (DFF[1] - DFF[0]);
        } else if (EC <= DFF[2]) {
            Dn    = -1;
            DF[0] = (DFF[2] - EC) / (DFF[2] - DFF[1]);
        } else if (EC <= DFF[3]) {
            Dn    = 0;
            DF[0] = (DFF[3] - EC) / (DFF[3] - DFF[2]);
        } else if (EC <= DFF[4]) {
            Dn    = 1;
            DF[0] = (DFF[4] - EC) / (DFF[4] - DFF[3]);
        } else if (EC <= DFF[5]) {
            Dn    = 2;
            DF[0] = (DFF[5] - EC) / (DFF[5] - DFF[4]);
        } else if (EC <= DFF[6]) {
            Dn    = 3;
            DF[0] = (DFF[6] - EC) / (DFF[6] - DFF[5]);
        }
    }

    else if (EC <= DFF[0]) {
        Dn    = -2;
        DF[0] = 1;
    } else if (EC >= DFF[6]) {
        Dn    = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];

    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0] = rule_p[Pn + 2][Dn + 2];
    Un[1] = rule_p[Pn + 3][Dn + 2];
    Un[2] = rule_p[Pn + 2][Dn + 3];
    Un[3] = rule_p[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) // 求小
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*同隶属函数输出语言值求大*/
    if (Un[0] == Un[1]) {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2]) {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3]) {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2]) {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3]) {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3]) {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1    = UF[0] * UFF[Un[0]];
    t2    = UF[1] * UFF[Un[1]];
    t3    = UF[2] * UFF[Un[2]];
    t4    = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    if (temp2 != 0)
        U = temp1 / temp2;
    else {
        U = 0;
    }
    //    temp1=PF[0]*UFF[Un[0]]+PF[1]*UFF[Un[1]]+PF[0]*UFF[Un[2]]+PF[1]*UFF[Un[3]]+DF[0]*UFF[Un[0]]+DF[0]*UFF[Un[1]]+DF[1]*UFF[Un[2]]+DF[0]*UFF[Un[3]];
    //    U=temp1;
    return U;
}

int rule_d[7] = {6, 5, 3, 2, 3, 5, 6};
float Kd_Fuzzy(float EC)
{
    float out         = 0;
    uint8 i           = 0;
    float degree_left = 0, degree_right = 0;
    uint8 degree_left_index = 0, degree_right_index = 0;

    float UFF[7];

    for (i = 0; i < 7; i++) {
        UFF[i] = kd_m / 6 * i;
    }

    if (EC < DFF[0]) {
        degree_left       = 1;
        degree_right      = 0;
        degree_left_index = 0;
    } else if (EC > DFF[6]) {
        degree_left       = 1;
        degree_right      = 0;
        degree_left_index = 6;
    } else {
        for (i = 0; i < 6; i++) {
            if (EC >= DFF[i] && EC < DFF[i + 1]) {
                degree_left        = (float)(DFF[i + 1] - EC) / (DFF[i + 1] - DFF[i]);
                degree_right       = 1 - degree_left;
                degree_left_index  = i;
                degree_right_index = i + 1;
                break;
            }
        }
    }

    out = UFF[rule_d[degree_left_index]] * degree_left + UFF[rule_d[degree_right_index]] * degree_right;

    return out;
}



 float KP_Fuzzy_w(float E, float EC)
{

    int rule_p[7][7] =
        {
            {6, 5, 4, 4, 3, 0, 0}, //-36
            {6, 4, 3, 3, 2, 0, 0}, //-24
            {4, 3, 2, 1, 0, 1, 2}, //-12
            {2, 1, 1, 0, 1, 1, 2}, // 0
            {2, 1, 0, 1, 2, 3, 4}, // 12
            {0, 0, 2, 3, 3, 4, 6}, // 24
            {0, 1, 3, 4, 4, 5, 6}, // 36
        };

    uint8 i2;


    float UFF[7];

    for (i2 = 0; i2 < 7; i2++) {
        UFF[i2] = kp_w / 6 * i2;
    }

    float U     = 0;
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};

    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;

    if (E > w_EFF[0] && E < w_EFF[6]) {
        if (E <= w_EFF[1]) {
            Pn    = -2;
            PF[0] = (w_EFF[1] - E) / (w_EFF[1] - w_EFF[0]);
        } else if (E <= w_EFF[2]) {
            Pn    = -1;
            PF[0] = (w_EFF[2] - E) / (w_EFF[2] - w_EFF[1]);
        } else if (E <= w_EFF[3]) {
            Pn    = 0;
            PF[0] = (w_EFF[3] - E) / (w_EFF[3] - w_EFF[2]);
        } else if (E <= w_EFF[4]) {
            Pn    = 1;
            PF[0] = (w_EFF[4] - E) / (w_EFF[4] - w_EFF[3]);
        } else if (E <= w_EFF[5]) {
            Pn    = 2;
            PF[0] = (w_EFF[5] - E) / (w_EFF[5] - w_EFF[4]);
        } else if (E <= w_EFF[6]) {
            Pn    = 3;
            PF[0] = (w_EFF[6] - E) / (w_EFF[6] - w_EFF[5]);
        }
    }

    else if (E <= w_EFF[0]) {
        Pn    = -2; /*  ??? */
        PF[0] = 1;
    } else if (E >= w_EFF[6]) {
        Pn    = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    if (EC > w_DFF[0] && EC < w_DFF[6]) {
        if (EC <= w_DFF[1]) {
            Dn    = -2;
            DF[0] = (w_DFF[1] - EC) / (w_DFF[1] - w_DFF[0]);
        } else if (EC <= w_DFF[2]) {
            Dn    = -1;
            DF[0] = (w_DFF[2] - EC) / (w_DFF[2] - w_DFF[1]);
        } else if (EC <= w_DFF[3]) {
            Dn    = 0;
            DF[0] = (w_DFF[3] - EC) / (w_DFF[3] - w_DFF[2]);
        } else if (EC <= w_DFF[4]) {
            Dn    = 1;
            DF[0] = (w_DFF[4] - EC) / (w_DFF[4] - w_DFF[3]);
        } else if (EC <= w_DFF[5]) {
            Dn    = 2;
            DF[0] = (w_DFF[5] - EC) / (w_DFF[5] - w_DFF[4]);
        } else if (EC <= w_DFF[6]) {
            Dn    = 3;
            DF[0] = (w_DFF[6] - EC) / (w_DFF[6] - w_DFF[5]);
        }
    }

    else if (EC <= w_DFF[0]) {
        Dn    = -2;
        DF[0] = 1;
    } else if (EC >= w_DFF[6]) {
        Dn    = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];

    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0] = rule_p[Pn + 2][Dn + 2];
    Un[1] = rule_p[Pn + 3][Dn + 2];
    Un[2] = rule_p[Pn + 2][Dn + 3];
    Un[3] = rule_p[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) // 求小
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*同隶属函数输出语言值求大*/
    if (Un[0] == Un[1]) {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2]) {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3]) {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2]) {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3]) {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3]) {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1    = UF[0] * UFF[Un[0]];
    t2    = UF[1] * UFF[Un[1]];
    t3    = UF[2] * UFF[Un[2]];
    t4    = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    if (temp2 != 0)
        U = temp1 / temp2;
    else {
        U = 0;
    }
    //    temp1=PF[0]*UFF[Un[0]]+PF[1]*UFF[Un[1]]+PF[0]*UFF[Un[2]]+PF[1]*UFF[Un[3]]+DF[0]*UFF[Un[0]]+DF[0]*UFF[Un[1]]+DF[1]*UFF[Un[2]]+DF[0]*UFF[Un[3]];
    //    U=temp1;
    return U;
}
float Kd_Fuzzy_w(float EC)
{
    float out         = 0;
    uint8 i           = 0;
    float degree_left = 0, degree_right = 0;
    uint8 degree_left_index = 0, degree_right_index = 0;

    float UFF[7];

    for (i = 0; i < 7; i++) {
        UFF[i] = kd_w / 6 * i;
    }

    if (EC < w_DFF[0]) {
        degree_left       = 1;
        degree_right      = 0;
        degree_left_index = 0;
    } else if (EC > w_DFF[6]) {
        degree_left       = 1;
        degree_right      = 0;
        degree_left_index = 6;
    } else {
        for (i = 0; i < 6; i++) {
            if (EC >= w_DFF[i] && EC < w_DFF[i + 1]) {
                degree_left        = (float)(w_DFF[i + 1] - EC) / (w_DFF[i + 1] - w_DFF[i]);
                degree_right       = 1 - degree_left;
                degree_left_index  = i;
                degree_right_index = i + 1;
                break;
            }
        }
    }

    out = UFF[rule_d[degree_left_index]] * degree_left + UFF[rule_d[degree_right_index]] * degree_right;

    return out;
}

float els = 0;
float elr = 0;
float pid_more(float error)
{
    float E = 0, EC = 0;
    static float32 err_last = 0;

    E  = error;
    EC = error - err_last;
		
    float output;

    fuzzy_kp    = KP_Fuzzy(E, EC) + fuzzy_kp_bas;
    fuzzy_kd    = Kd_Fuzzy(EC);
		
    fuzzy_P_out = fuzzy_kp * E;
    fuzzy_D_out = fuzzy_kd * EC;
	  fuzzy_I_out +=  fuzzy_ki * E;
    LimitMax(fuzzy_P_out,300);
    LimitMax(fuzzy_D_out,200);
		LimitMax(fuzzy_I_out,30);
		if(fabs(E)<6)fuzzy_I_out = 0;
    output   = -(fuzzy_P_out + fuzzy_D_out + fuzzy_I_out);
    err_last = error;
		els = EC;
		return output;
		
}
float w_els = 0;
float pid_more_w(float error)
{
    float E = 0, EC = 0;
    static float32 w_err_last = 0;

    E  = error;
    EC = error - w_err_last;
		
    float output;

    w_kp    = KP_Fuzzy_w(E, EC) + w_kp_bas;
    w_kd    = Kd_Fuzzy_w(EC);
		
    w_P_out = w_kp * E;
    w_D_out = w_kd * EC;
	  w_I_out +=  w_ki * E;
    LimitMax(w_P_out,300);
    LimitMax(w_D_out,200);
		LimitMax(w_I_out,50);
		if(fabs(E)<8)w_I_out = 0;
		if(fabs(E)<4)w_P_out *= 0.5;
    output   = -(w_P_out + w_D_out + w_I_out);
    w_err_last = error;
		w_els = EC;
		return output;
		
}


 float KP_Fuzzy_y(float E, float EC)
{

    int rule_p[7][7] =
        {
            {6, 5, 4, 4, 3, 0, 0}, //-36
            {6, 4, 3, 3, 2, 0, 0}, //-24
            {4, 3, 2, 1, 0, 1, 2}, //-12
            {2, 1, 1, 0, 1, 1, 2}, // 0
            {2, 1, 0, 1, 2, 3, 4}, // 12
            {0, 0, 2, 3, 3, 4, 6}, // 24
            {0, 1, 3, 4, 4, 5, 6}, // 36
        };

    uint8 i2;


    float UFF[7];

    for (i2 = 0; i2 < 7; i2++) {
        UFF[i2] = kp_y / 6 * i2;
    }

    float U     = 0;
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0};

    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;

    if (E > y_EFF[0] && E < y_EFF[6]) {
        if (E <= y_EFF[1]) {
            Pn    = -2;
            PF[0] = (y_EFF[1] - E) / (y_EFF[1] - y_EFF[0]);
        } else if (E <= y_EFF[2]) {
            Pn    = -1;
            PF[0] = (y_EFF[2] - E) / (y_EFF[2] - y_EFF[1]);
        } else if (E <= y_EFF[3]) {
            Pn    = 0;
            PF[0] = (y_EFF[3] - E) / (y_EFF[3] - y_EFF[2]);
        } else if (E <= y_EFF[4]) {
            Pn    = 1;
            PF[0] = (y_EFF[4] - E) / (y_EFF[4] - y_EFF[3]);
        } else if (E <= y_EFF[5]) {
            Pn    = 2;
            PF[0] = (y_EFF[5] - E) / (y_EFF[5] - y_EFF[4]);
        } else if (E <= y_EFF[6]) {
            Pn    = 3;
            PF[0] = (y_EFF[6] - E) / (y_EFF[6] - y_EFF[5]);
        }
    }

    else if (E <= y_EFF[0]) {
        Pn    = -2; /*  ??? */
        PF[0] = 1;
    } else if (E >= y_EFF[6]) {
        Pn    = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    if (EC > y_DFF[0] && EC < y_DFF[6]) {
        if (EC <= y_DFF[1]) {
            Dn    = -2;
            DF[0] = (y_DFF[1] - EC) / (y_DFF[1] - y_DFF[0]);
        } else if (EC <= y_DFF[2]) {
            Dn    = -1;
            DF[0] = (y_DFF[2] - EC) / (y_DFF[2] - y_DFF[1]);
        } else if (EC <= y_DFF[3]) {
            Dn    = 0;
            DF[0] = (y_DFF[3] - EC) / (y_DFF[3] - y_DFF[2]);
        } else if (EC <= y_DFF[4]) {
            Dn    = 1;
            DF[0] = (y_DFF[4] - EC) / (y_DFF[4] - y_DFF[3]);
        } else if (EC <= y_DFF[5]) {
            Dn    = 2;
            DF[0] = (y_DFF[5] - EC) / (y_DFF[5] - y_DFF[4]);
        } else if (EC <= y_DFF[6]) {
            Dn    = 3;
            DF[0] = (y_DFF[6] - EC) / (y_DFF[6] - y_DFF[5]);
        }
    }

    else if (EC <= y_DFF[0]) {
        Dn    = -2;
        DF[0] = 1;
    } else if (EC >= y_DFF[6]) {
        Dn    = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];

    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0] = rule_p[Pn + 2][Dn + 2];
    Un[1] = rule_p[Pn + 3][Dn + 2];
    Un[2] = rule_p[Pn + 2][Dn + 3];
    Un[3] = rule_p[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) // 求小
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];
    /*同隶属函数输出语言值求大*/
    if (Un[0] == Un[1]) {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2]) {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3]) {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2]) {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3]) {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3]) {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1    = UF[0] * UFF[Un[0]];
    t2    = UF[1] * UFF[Un[1]];
    t3    = UF[2] * UFF[Un[2]];
    t4    = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    if (temp2 != 0)
        U = temp1 / temp2;
    else {
        U = 0;
    }
    //    temp1=PF[0]*UFF[Un[0]]+PF[1]*UFF[Un[1]]+PF[0]*UFF[Un[2]]+PF[1]*UFF[Un[3]]+DF[0]*UFF[Un[0]]+DF[0]*UFF[Un[1]]+DF[1]*UFF[Un[2]]+DF[0]*UFF[Un[3]];
    //    U=temp1;
    return U;
}

float Kd_Fuzzy_y(float EC)
{
    float out         = 0;
    uint8 i           = 0;
    float degree_left = 0, degree_right = 0;
    uint8 degree_left_index = 0, degree_right_index = 0;

    float UFF[7];

    for (i = 0; i < 7; i++) {
        UFF[i] = kd_y / 6 * i;
    }

    if (EC < y_DFF[0]) {
        degree_left       = 1;
        degree_right      = 0;
        degree_left_index = 0;
    } else if (EC > y_DFF[6]) {
        degree_left       = 1;
        degree_right      = 0;
        degree_left_index = 6;
    } else {
        for (i = 0; i < 6; i++) {
            if (EC >= y_DFF[i] && EC < y_DFF[i + 1]) {
                degree_left        = (float)(y_DFF[i + 1] - EC) / (y_DFF[i + 1] - y_DFF[i]);
                degree_right       = 1 - degree_left;
                degree_left_index  = i;
                degree_right_index = i + 1;
                break;
            }
        }
    }

    out = UFF[rule_d[degree_left_index]] * degree_left + UFF[rule_d[degree_right_index]] * degree_right;

    return out;
}

float y_els = 0;
float pid_more_y(float error)
{
    float E = 0, EC = 0;
    static float32 y_err_last = 0;

    E  = error;
    EC = error - y_err_last;
		
    float output;

    y_kp    = KP_Fuzzy_y(E, EC) + y_kp_bas;
    y_kd    = Kd_Fuzzy_y(EC);
		
    y_P_out = y_kp * E;
    y_D_out = y_kd * EC;
	  y_I_out +=  y_ki * E;
    LimitMax(y_P_out,300);
    LimitMax(y_D_out,200);
		LimitMax(y_I_out,30);
		if(fabs(E)<6)y_I_out = 0;
    output   = -(y_P_out + y_D_out + y_I_out);
    y_err_last = error;
		y_els = EC;
		return output;
}
