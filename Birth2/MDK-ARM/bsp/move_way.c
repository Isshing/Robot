#include "IMU.h"
#include "move_way.h"
#include "CAN_receive.h"
extern float vx;
extern float vy;
extern float vw;
float base_distance = 183;
int test_flag = 0;
#define TOF_y TOF2
#define TOF_x TOF1
void move_to_desk()
{
    static bool_t moveToPosition = 0;
    const int distanceThresholdY = 850; 
    const int distanceThresholdX = 1530;
    const int speedTowardsY = 350;
    const int speedTowardsX = 350;
    if (!moveToPosition)
    {
        vx = 0;
        vy = speedTowardsY; 
        if (TOF_y >= distanceThresholdY)
        {
            moveToPosition = 1; 
        }
    }
    else
    {
        vx = speedTowardsX; 
        vy = 0;
        if (TOF_x >= distanceThresholdX)
        {
            vx = 0;
            vy = 0;
            //test_flag = 1; 
        }
    }
}

void move_to_container()
{
    static bool_t moveToPosition = 0;
    const int distanceThresholdY = 600; 
    const int distanceThresholdX = 600;
    const int speedTowardsY = -350;
    const int speedTowardsX = 350;


    if (!moveToPosition)
    {
        vx = 0;
        vy = speedTowardsY;
        if (TOF1 >= distanceThresholdY)
        {
            moveToPosition = 1; 
        }
    }
    else
    {
        vx = speedTowardsX; 
        vy = 0;
        if (TOF2 >= distanceThresholdX)
        {
            test_flag = 2;
        }
    }
}

void move_to_search()
{
    static bool_t moveToPosition = 0;
    const int distanceThresholdX = 600;
    const int distanceThresholdX_low = 600;
    const int speedTowardsX = 350;
    // �����һ��Ŀ��㣨��Y����
    if (!moveToPosition)
    {
        vx = speedTowardsX;
        if (TOF2 >= distanceThresholdX)
        {
            moveToPosition = 1 - moveToPosition;
        }
    }
    else
    {
        // ����ڶ���Ŀ��㣨��X����
        vx = -speedTowardsX;
        if (TOF2 <= distanceThresholdX_low)
        {
            moveToPosition = 1 - moveToPosition;
        }
    }
}

void test_move()
{
    if (test_flag == 0)
    { //
        move_to_desk();
    }
    else if (test_flag == 1)
    {
        move_to_container();
    }
    else if (test_flag == 2)
    {
        move_to_search();
    }
}