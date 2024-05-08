
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
extern unsigned int jeston_flag ;
extern uint16_t distance;
#define TOF_x TOF2
#define TOF_y TOF1
int met = 0;
int CR_flag = 0;
int N_flag = 0;
#define sg_down 1
#define sg_up 0
#define level1 40
#define level2 240
#define level3 900
void move_to_desk()
{
    static bool_t moveToPosition = 0;
    int distanceThresholdX = 985; 
    int distanceThresholdY = 1450;
    int speedTowardsY = 300;
    int speedTowardsX = 300;
		up_move(level2,0);
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
					if(met == 0){
						CR_flag = 1;
						met = 1;
					}
        }
    }
}
int turn_ward = 0;
int waiting_up = 0;
int waiting_counter = 0;

void move_to_container()
{
    static bool_t moveToPosition = 0;
    int distanceThresholdY_H = 1700; 
		int distanceThresholdY_L = 870; 
    int distanceThresholdX = 280;
    int speedTowardsY = 300;
    int speedTowardsX = 300;
		if(!moveToPosition){
			up_move(level2,1);
			if((TOF_x >= distanceThresholdX)&&(moveToPosition==0)){
				vx = -speedTowardsX;
				vy = 0;
			}else{
				vx = 0;
				if(TOF_y<distanceThresholdY_H){
					vy = speedTowardsY;
				}else{
					vy = 0;
					if(met == 0){
						N_flag = 1;
						met = 1;
					}
					if(jeston_flag == 5)moveToPosition = 1;
				}
			}
		}
    else if (moveToPosition == 1)
    {			
			if(jeston_flag == 2){//jeston command to "Stop"
				vx = 0;
			  vy = 0;
			}else{
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
				if(waiting_up == 2&& half_move==0)moveToPosition = 2;
				if(waiting_up == 3&& half_move==1)moveToPosition = 4;
			}
    }else if(moveToPosition == 2){
				if(TOF1<= 1980){
					go_to_roll = 1;
				}else{
					vx = 0;
					vy = 0;
					initial_flag = 0;
					moveToPosition = 3;
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
			}else if(moveToPosition == 3){//ending
				if(rolling_flag == 0){	
					go_to_roll = 0;
					if(walling_start == 1){
						if(TOF_x>900){
							vx = -speedTowardsX;
						}else{
							vx = 0;
							moveToPosition = 1;
						}
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
uint8 up_done_flag = 0;
void up_move(uint16 high,uint16 direction){ 
	if(direction == 1){
			if(high < distance){
				CAN_cmd_up(0x01, 0x01, 0x20, 0x00, 0x00, 0x01, 0x00);//p2 0x01 DOWN Ox00 UP   //32-25.9-32  2000
				up_done_flag = 0;
			}else{
				CAN_cmd_up(0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00);//p2 0x01 DOWN Ox00 UP
				up_done_flag = 1;
			}
	}else{
			if(high > distance){
				CAN_cmd_up(0x01, 0x00, 0x20, 0x00, 0x00, 0x01, 0x00);//p2 0x01 DOWN Ox00 UP   //32-25.9-32  2000
				up_done_flag = 0;
			}else{
				CAN_cmd_up(0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00);//p2 0x01 DOWN Ox00 UP
				up_done_flag = 1;
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
