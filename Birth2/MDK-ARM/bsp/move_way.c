#include "IMU.h"
#include "move_way.h"
#include "CAN_receive.h"
extern float vx;
extern float vy;
extern float vw;
float base_distance = 183;
int test_flag = 0;
int half_move = 0;
#define TOF_x TOF3
#define TOF_y TOF1
void move_to_desk()
{
    static bool_t moveToPosition = 0;
    int distanceThresholdX = 870; 
    int distanceThresholdY = 1450;
    int speedTowardsY = 300;
    int speedTowardsX = 300;
    if (!moveToPosition)
    {
        vx = speedTowardsX; 
        vy = 0;
			   if (TOF_x >= distanceThresholdX)
        {
            moveToPosition = 1; 
        }
    }
    else
    { 
			  vx = 0;
        vy = speedTowardsY; 

        if (TOF_y >= distanceThresholdY)
        {
            vx = 0;
            vy = 0;
            test_flag = 1; 
        }
    }
}
int turn_ward = 0;
int waiting_up = 0;
int waiting_counter = 0;
void move_to_container()
{
    static bool_t moveToPosition = 0;
    int distanceThresholdY_H = 1500; 
		int distanceThresholdY_L = 870; 
    int distanceThresholdX = 250;
    int speedTowardsY = 300;
    int speedTowardsX = 300;
		
		if(!moveToPosition){
			if((TOF_x >= distanceThresholdX)&&(moveToPosition==0)){
				vx = -speedTowardsX;
				vy = 0;
			}else{
				vx = 0;
				moveToPosition = 1;
			}
		}
    else if (moveToPosition == 1)
    {
			if(turn_ward == 0){
				vy = -speedTowardsY;
				if(TOF_y<= distanceThresholdY_L){
					waiting_up++;
					turn_ward = 1;
				}
			}else{
				vy = speedTowardsY;
				if(TOF_y>= distanceThresholdY_H){
					waiting_up++;
					turn_ward = 0;
				}
			}
			if(waiting_up == 3&& half_move==0)moveToPosition = 2;
			if(waiting_up == 4&& half_move==1)moveToPosition = 3;
    }else if(moveToPosition == 2){
				half_move = 1;
				if(TOF1<= 375){
					vx = speedTowardsX;
					vy = 0;
				}
				if(TOF2>2300){
					vx = 0;
					moveToPosition = 1;
					waiting_up = 0;
				}
				//waiting_counter ++;
				//if(waiting_counter>10000){moveToPosition = 1;}
		}else if(moveToPosition == 3){//ending
				if(TOF1<= 375){
					if(TOF2>= 2880){
						vx = 0;
						vy = 0;
						vw = 0;
					}else{
						vx = speedTowardsX;
						vy = 0;
					}
				}
		}
		
}

void move_to_search()
{
    static bool_t moveToPosition = 0;
    const int distanceThresholdX = 600;
    const int distanceThresholdX_low = 600;
    const int speedTowardsX = 350;
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
    {
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
