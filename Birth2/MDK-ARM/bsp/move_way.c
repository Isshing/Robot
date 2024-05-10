
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
int level[4];
uint8 up_done_flag = 0;
uint8 last_up_done_flag = 0;
int tram = 0;
void up_move(int high,int lop){
		if(up_done_flag == 0)tram = 1;
		if(distance>high+13){
			CAN_cmd_up(0x01, 0x01, 0x20, 0x00, 0x00, 0x01, 0x00);//p2 0x01 DOWN Ox00 UP   //32-25.9-32  2000
			up_done_flag = 0;
		}else if(distance<high-13){
			CAN_cmd_up(0x01, 0x00, 0x20, 0x00, 0x00, 0x01, 0x00);//p2 0x01 DOWN Ox00 UP   //32-25.9-32  2000
			up_done_flag = 0;
		}else{
			CAN_cmd_up(0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00);//p2 0x01 DOWN Ox00 UP
			up_done_flag = 1;
		}
		if(up_done_flag == 1&&test_flag!= 0&&tram == 1){
			if(lop == 1){
				HAL_UART_Transmit(&huart7, (uint8_t *)"AP1B", strlen("AP1B"), 999);
				tram = 0;
			}else if(lop == 2){
				HAL_UART_Transmit(&huart7, (uint8_t *)"AP2B", strlen("AP2B"), 999);		
				tram = 0;
			}else if(lop == 3){
				HAL_UART_Transmit(&huart7, (uint8_t *)"AP3B", strlen("AP3B"), 999);	
				tram = 0;
			}
		}
}
void tof_mvoe2(int tof_dis,int target_dis,int speed_dis,int tof_number){
	if(tof_number == 1 || tof_number == 4){//y
		if(tof_dis< target_dis-8){
			vy = speed_dis* (1 - 2*half_move);
		}else if(tof_dis >target_dis+8){
			vy = -speed_dis* (1 - 2*half_move);
		}else{
			vy =0;
		}
	}else{//x
		if(tof_dis< target_dis-20){
			vx =speed_dis* (1 - 2*half_move);
		}else if(tof_dis >target_dis+20){
			vx = -speed_dis* (1 - 2*half_move);
		}else{
			vx =0;
		}
	}
}
void move_to_desk2()
{
    static bool_t moveToPosition = 0;
    int distanceThresholdX = 985; 
    int distanceThresholdY = 1450;
    int speedTowardsY = 300;
    int speedTowardsX = 300;
    if (!moveToPosition)
    {
			if(TOF_x<distanceThresholdX - 200){
				tof_mvoe2(TOF_x,distanceThresholdX - 200,speedTowardsX + 300,2);
			}else if(TOF_x<460){
				tof_mvoe2(TOF_x,distanceThresholdX - 200,speedTowardsX,2);
			}
			else{
				tof_mvoe2(TOF_x,distanceThresholdX+20,speedTowardsX,2);
			}
			if(TOF_x>680){
				vy = 500;
			}else{
				vy = 0;
			}
			if (TOF_x >= distanceThresholdX)
			{
					moveToPosition = 1; 
			}
    }
    else
    { 
			  vx = 0;
				if(TOF_y<distanceThresholdY - 300 && TOF_y> 500){
					vy = speedTowardsY + 300;
					tof_mvoe2(TOF_y,distanceThresholdY - 200,speedTowardsY + 300,1);					
				}else{
					vy = speedTowardsY;
					tof_mvoe2(TOF_y,distanceThresholdY +20,speedTowardsY,1);
				}
        if (TOF_y >= distanceThresholdY)
        {
					vx = 0;
					vy = 0;
					vw = 0;
					if(met == 0){
						CR_flag = 1;
						met = 1;
					}
					up_done_flag = 0;
					test_flag = 1;
        }
    }
}
int turn_ward = 0;
int waiting_up = 0;
int waiting_counter = 0;
int vv = 0;
void move_to_container2()
{
	  level[0] = 0;
		level[1] = level1;
		level[2] = level2;
		level[3] = level3;
    static bool_t moveToPosition = 0;
    int distanceThresholdY_H = 1700; 
		int distanceThresholdY_L = 630; 
    int distanceThresholdX = 280;
    int speedTowardsY = 300;
    int speedTowardsX = 300;
		if(!moveToPosition){
			up_move(level1,1);
			if(TOF_x >= distanceThresholdX){
				tof_mvoe2(TOF_x,distanceThresholdX,speedTowardsX,2);
				vy = 0;
			}else{
				vx = 0;
				if(TOF_y<distanceThresholdY_H){
					tof_mvoe2(TOF_y,distanceThresholdY_H,speedTowardsY,1);
				}else{
					vy = 0;
					if(vv == 0){
						HAL_UART_Transmit(&huart7, (uint8_t *)"ALLB", strlen("ALLB"), 999);
						vv = 1;
					}
					if(up_done_flag == 1){
						moveToPosition = 1;
					}
				}
			}
		}
    else if (moveToPosition == 1)
    {			
			if(jeston_flag == 2){//jeston command to "Stop"
				vx = 0;
			  vy = 0;
			}else{
				vv = 0;
			if(turn_ward == 0){
				if(up_done_flag == 1){
					vy = -speedTowardsY * (1 - 2*half_move);
				}else{
					vy = 0;
					if(waiting_up <3)up_move(level[waiting_up+1],waiting_up+1);
				}
				if(TOF_y< distanceThresholdY_L){
					waiting_up++;
					if(vv == 0){
						HAL_UART_Transmit(&huart7, (uint8_t *)"ASGB", strlen("ASGB"), 999);
						vv = 1;
					}
					turn_ward = 1;
					up_done_flag = 0;
				}
			}else{
				if(up_done_flag == 1){
					vy = speedTowardsY * (1 - 2*half_move);
				}else{
					vy = 0;
					if(waiting_up <3)up_move(level[waiting_up+1],waiting_up+1);
				}
				if(TOF_y> distanceThresholdY_H){
						waiting_up++;
						if(vv == 0){
							HAL_UART_Transmit(&huart7, (uint8_t *)"ASGB", strlen("ASGB"), 999);
							vv = 1;
						}	
						turn_ward = 0;
					  up_done_flag = 0;
					}
				}
				if(waiting_up == 4&& half_move==0){
					moveToPosition = 2;
				}
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
						up_move(level1,1);
						if(TOF_x>700){
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
