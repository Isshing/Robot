/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
extern uint16 TOF1;
extern uint16 TOF2;
extern uint16 TOF3;
extern uint16 TOF4;

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_PINTAI_ID = 0x005,
    CAN_TOF_ID = 0x601,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern motor_measure_t motor_chassis[4];

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);


extern void CAN_cmd_chassis_reset_ID(void);



extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);


extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);


extern const motor_measure_t *get_trigger_motor_measure_point(void);


extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern void CAN_cmd_up(int8_t mode, int8_t toward, int8_t resolution, int8_t POS_H,int8_t POS_L,int8_t SPEED_H,int8_t SPEED_L);


#endif
