/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ??????CAN?��???????????????????,CAN???????????????????????.
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

#include "CAN_receive.h"
#include "main.h"
#include "pid.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
???????, 0:??????1 3508???,  1:??????2 3508???,2:??????3 3508???,3:??????4 3508???;
4:yaw?????? 6020???; 5:pitch?????? 6020???; 6:??????? 2006???*/
motor_measure_t motor_chassis[4];
uint16 TOF1 = 0;
uint16 TOF2 = 0;
uint16 TOF3 = 0;
uint16 TOF4 = 0;
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef up_tx_message;
static uint8_t up_can_send_data[7];
/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal??CAN???????,??????????
 * @param[in]      hcan:CAN??????
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId)
  {
  case CAN_3508_M1_ID:
  case CAN_3508_M2_ID:
  case CAN_3508_M3_ID:
  case CAN_3508_M4_ID:
  {
    static uint8_t i = 0;
    // get motor id
    i = rx_header.StdId - CAN_3508_M1_ID;
    get_motor_measure(&motor_chassis[i], rx_data);
		motor_chassis[i].speed_rpm = movingAverageFilter(i, motor_chassis[i].speed_rpm);
		
    break;
  }
  case CAN_TOF_ID:
  {
    TOF1 = (uint16_t)(rx_data[0]<<8 | rx_data[1]);
    TOF2 = (uint16_t)(rx_data[2]<<8 | rx_data[3]);
    TOF3 = (uint16_t)(rx_data[4]<<8 | rx_data[5]);
    TOF4 = (uint16_t)(rx_data[6]<<8 | rx_data[7]);
    break;
  }
  default:
  {
    break;
  }
  }
}



void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_up(int8_t mode, int8_t toward, int8_t resolution, int8_t POS_H, int8_t POS_L, int8_t SPEED_H, int8_t SPEED_L)
{
  uint32_t send_mail_box;
  up_tx_message.StdId = 0x01; // CAN_CHASSIS_ALL_ID;
  up_tx_message.IDE = CAN_ID_STD;
  up_tx_message.RTR = CAN_RTR_DATA;
  up_tx_message.DLC = 0x08;
  up_can_send_data[0] = mode;
  up_can_send_data[1] = toward;
  up_can_send_data[2] = resolution;
  up_can_send_data[3] = POS_H;
  up_can_send_data[4] = POS_L;
  up_can_send_data[5] = SPEED_H;
  up_can_send_data[6] = SPEED_L;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &up_tx_message, up_can_send_data, &send_mail_box);
}



const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
