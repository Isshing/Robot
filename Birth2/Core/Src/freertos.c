/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include <string.h>
#include <stdio.h>
#include "CAN_receive.h"
#include "pid.h"
#include "IMU.h"
#include "move_way.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern unsigned char uart8Rx[32];
extern unsigned char uart6Tx[32];
extern unsigned char uart7Rx[32];
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern unsigned int jeston_flag;
extern uint8_t RxByte;
extern void ANO_sent_data(int16 A, int16 B, int16 C, int16 D, int16 E, int16 F, int16 G, int16 H, int16 I, int16 J);
extern pid_type_def motor_pid_0,motor_pid_1,motor_pid_2,motor_pid_3,angle_pid;		
float vx = 0;
float vy = 0;
float vw = 0;
int set_v,set_spd[4];
extern uint8_t uart6Rx[32];          
extern uint16_t uart6RxLength;
extern float set_speed_0,set_speed_1,set_speed_2,set_speed_3;
extern motor_measure_t *motor_data_0,*motor_data_1,*motor_data_2,*motor_data_3;
extern int test_flag;
extern char jetson_data[2];
extern void Jetson_read(unsigned char *data);
int st = 0;
int ready = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId PID_ControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


void move_solution(float vx, float vy, float vw){
  static float rotate_ratio_f = 1.;
  static float wheel_rpm_ratio = 1.;
  static float rotate_ratio_b = 1.;
    //wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO); //mm/s = 60/(???????*?????(19??1)) rpm
    //rotate_ratio_f =  (WHEELBASE+WHEELTRACK)/2.0f/RADIAN_COEF; //(rad/s)/57.3 = deg/s
    set_speed_0 = (-vy + vx + vw * rotate_ratio_f) * wheel_rpm_ratio;
    set_speed_1 = (vy + vx + vw * rotate_ratio_b) * wheel_rpm_ratio;
    set_speed_2 = (vy - vx + vw * rotate_ratio_f) * wheel_rpm_ratio;
    set_speed_3 = (-vy - vx + vw * rotate_ratio_b) * wheel_rpm_ratio;
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void PID_Control_Function(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PID_Control */
  osThreadDef(PID_Control, PID_Control_Function, osPriorityIdle, 0, 256);
  PID_ControlHandle = osThreadCreate(osThread(PID_Control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  for (;;)
  {
    // char buffer8[32];
    // char buffer7[32];
    // sprintf(buffer7, "%lu\r\n", TOF_distance7);
    // sprintf(buffer8, "%lu\r\n", TOF_distance8);
    //		HAL_UART_Transmit((UART_HandleTypeDef *)&huart6, (uint8_t *)"TOFuart8:", (uint16_t)strlen("TOFuart8:"), (uint32_t)999);
    //		HAL_UART_Transmit(&huart6, (uint8_t *)buffer8, strlen(buffer8), 999);
    //ANO_sent_data(motor_data_0->speed_rpm, motor_data_1->speed_rpm, motor_data_2->speed_rpm, motor_data_3->speed_rpm, (int16)vw,(int16)heading_deg, set_speed_0, set_speed_1,set_speed_2 , set_speed_3);
		//ANO_sent_data(motor_data_0->speed_rpm, set_speed_0,(int16)motor_pid_0.Pout,(int16)gein, (int16)bss,(int16)motor_pid_0.Iout, (int16)motor_pid_0.Ki,(int16)motor_pid_0.Kp ,(int16)motor_pid_0.Pout ,(int16)motor_pid_0.Dout);
		//ANO_sent_data(motor_data_0->speed_rpm, set_speed_0,(int16)kpdata,(int16)kidata, (int16)kddata,(int16)outdata, 0,0 ,0,0);
		//ANO_sent_data(motor_data_0->speed_rpm,(int16)set_speed_0, (int16)motor_pid_0.out,(int16)motor_pid_0.Pout, (int16)motor_pid_0.Iout,(int16)motor_pid_0.Dout, (int16)motor_pid_0.error[0],0 ,0,0);
			//float TOF1 4
			if(jetson_data[0] == 'O' && jetson_data[1] == 'K'){
				jeston_flag = 1;
				ready = 1;
				memset(uart6Rx,0,sizeof(uart6Rx));
				memset(jetson_data,0,sizeof(jetson_data));
			}
			else if(jetson_data[0] == 'S' && jetson_data[1] == 'T'){
				jeston_flag = 2;
				st = 1;
			}else if(jetson_data[0] == 'R' && jetson_data[1] == 'G'){
				jeston_flag = 3;
				st = 1;
			}
			if(st == 1){
				st = 0;
				memset(uart6Rx,0,sizeof(uart6Rx));
				memset(jetson_data,0,sizeof(jetson_data));
				HAL_UART_Transmit(&huart6, (uint8_t *)"AOKB", strlen("AOKB"), 999);
			}
		TTL_Hex2Dec();
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_PID_Control_Function */
/**
 * @brief Function implementing the PID_Control thread.=
 * @param argument: Not used 
 * @retval None
 */
/* USER CODE END Header_PID_Control_Function */
int turning = 0;
void PID_Control_Function(void const * argument)
{
  /* USER CODE BEGIN PID_Control_Function */
  /* Infinite loop */
  for (;;)
  {
		//vx = 400;
		if(initial_flag == 1){
//			Jetson_read(uart6Rx);
//			if(jeston_flag == 0){
//				vy = 400;
//				if(TOF1<50&&(ready ==0)){HAL_UART_Transmit(&huart6, (uint8_t *)"ANB", strlen("ANB"), 999);}			
//			}else if(jeston_flag == 1){
//				vy = -300;
//			}else if(jeston_flag == 2){
//				vy = 0;
//			}else if(jeston_flag == 3){
//				vy = -300;
//			}
			vy = 300; // 1heng
			//vw = PID_calc(&angle_pid, heading_deg,initial_angle);
			move_solution(vx,vy,vw);
			PID_calc(&motor_pid_0, motor_data_0->speed_rpm, set_speed_0); 
			PID_calc(&motor_pid_1, motor_data_1->speed_rpm, set_speed_1); 
			PID_calc(&motor_pid_2, motor_data_2->speed_rpm, set_speed_2); 
			PID_calc(&motor_pid_3, motor_data_3->speed_rpm, set_speed_3); 
			CAN_cmd_chassis(motor_pid_0.out,motor_pid_1.out, motor_pid_2.out, motor_pid_3.out); 
		}
    osDelay(2);
  }
  /* USER CODE END PID_Control_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
