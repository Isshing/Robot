#ifndef __MOVE_WAY_H__
#define __MOVE_WAY_H__ 
#include "can.h"
#include <string.h>
#include <stdio.h>
#include "CAN_receive.h"
#include "pid.h"
#include "IMU.h"

#define sg_down 1
#define sg_up 0
#define level1 40
#define level2 240
#define level3 900

void up_move2(int high);
void move_to_desk(void);
void move_to_container(void);
//void up_move(int ,uint16);

#endif

