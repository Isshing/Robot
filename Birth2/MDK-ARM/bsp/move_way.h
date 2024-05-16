#ifndef __MOVE_WAY_H__
#define __MOVE_WAY_H__ 
#include "can.h"
#include <string.h>
#include <stdio.h>
#include "CAN_receive.h"
#include "pid.h"
#include "IMU.h"
#include "Steering.h"
#define sg_down 1
#define sg_up 0
#define level1 48
#define level2 245
#define level3 535
#define level4 550
void tof_mvoe2(int tof_dis,int target_dis,int speed_dis,int tof_number);
void move_to_desk(void);
void move_to_container(void);
void move_to_desk2(void);
void move_to_container2(void);
void up_move(int high,int lop);
#endif

