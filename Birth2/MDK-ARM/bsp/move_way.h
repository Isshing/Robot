#ifndef __MOVE_WAY_H__
#define __MOVE_WAY_H__ 
#include "can.h"
#include <string.h>
#include <stdio.h>
#include "CAN_receive.h"
#include "pid.h"
#include "IMU.h"

void move_to_desk(void);
void move_to_container(void);
void up_move(uint16 ,uint16 );
#endif

