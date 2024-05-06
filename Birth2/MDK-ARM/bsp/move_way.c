#include "can.h"
#include <string.h>
#include <stdio.h>
#include "CAN_receive.h"
#include "pid.h"
#include "IMU.h"
#include "move_way.h"
extern UART_HandleTypeDef huart8,huart7,huart6,huart2;
extern float vx;
extern float vy;
extern float vw;
extern int rolling_flag;
extern unsigned int jeston_flag;
extern int go_to_roll;
float base_distance = 183;
int test_flag = 0;
int half_move = 0;
int walling_start = 0;
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
						vw = 0;
						HAL_UART_Transmit(&huart6, (uint8_t *)"ACRB", strlen("ACRB"), 999);
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
				HAL_UART_Transmit(&huart6, (uint8_t *)"ANB", strlen("ANB"), 999);
				moveToPosition = 1;
			}
		}
    else if (moveToPosition == 1)
    {
			if(turn_ward == 0){
				vy = -speedTowardsY * (1 - 2*half_move);
				if(TOF_y<= distanceThresholdY_L){
					waiting_up++;
					turn_ward = 1;
				}
			}else{
				vy = speedTowardsY * (1 - 2*half_move);
				if(TOF_y>= distanceThresholdY_H){
					waiting_up++;
					turn_ward = 0;
				}
			}
			if(jeston_flag == 2){//jeston command to "Stop"
				vx = 0;
			  vy = 0;
			}
			if(waiting_up == 2&& half_move==0)moveToPosition = 2;
			if(waiting_up == 3&& half_move==1)moveToPosition = 4;
    }else if(moveToPosition == 2){
				if(TOF1<= 1980){
					go_to_roll = 1;
				}else{
					vx = 0;
					vy = 0;
					initial_flag = 0;
					moveToPosition = 5;
					rolling_flag = 1;
					half_move = 1;
				}
//			}else if(moveToPosition == 3){
//				if(rolling_flag == 0){
//					go_to_roll = 0;
//					vx = -speedTowardsX;
//					vy = 0;
//					if(TOF2<870){
//						moveToPosition = 1;
//					}
//				}
//			}else if(moveToPosition == 4){//ending
//				if(TOF_x>= 2200){
//					if(TOF_y<= 500){
//						vx = 0;
//						vy = 0;
//						vw = 0;
//					}else{
//						vx = -speedTowardsX;
//						vy = 0;
//					}
			}else if(moveToPosition == 5){//ending
				if(rolling_flag == 0){	
					go_to_roll = 0;
					if(walling_start == 1){
						vx = -speedTowardsX;
					}else{
						if(TOF_y>300){
							vy = -speedTowardsY;
						}else{
							vy = 0;
							walling_start = 1;
						}
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
