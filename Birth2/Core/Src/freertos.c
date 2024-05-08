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
extern unsigned char uart8Rx[32],uart6Tx[32],uart7Rx[32];
extern UART_HandleTypeDef huart8,huart7,huart6,huart2;
extern unsigned int jeston_flag;
extern uint8_t RxByte;
extern void ANO_sent_data(int16 A, int16 B, int16 C, int16 D, int16 E, int16 F, int16 G, int16 H, int16 I, int16 J);
extern pid_type_def motor_pid_0,motor_pid_1,motor_pid_2,motor_pid_3,angle_pid,rof_pid;		
float vx,vy,vw = 0;
int set_v,set_spd[4];
extern uint8_t uart6Rx[32];          
extern uint16_t uart6RxLength;
extern float set_speed_0,set_speed_1,set_speed_2,set_speed_3;
extern motor_measure_t *motor_data_0,*motor_data_1,*motor_data_2,*motor_data_3;
extern int test_flag;
extern char jetson_data[2];
extern void Jetson_read(unsigned char *data);
int st = 0;
float error_tof_y,error_tof_x = 0;
extern int waiting_up,half_move;
int rolling_flag =0;
float turning_angle = 0;
int go_to_roll = 0;
float comp_angle = 0;
int stt = 3;
extern int CR_flag,N_flag;
int crmm = 3;
char last_jetson_data[2];
extern uint16_t distance;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId PID_ControlHandle;
osThreadId Move_controlHandle;

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
void Move_control_task(void const * argument);

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

  /* definition and creation of Move_control */
  osThreadDef(Move_control, Move_control_task, osPriorityIdle, 0, 256);
  Move_controlHandle = osThreadCreate(osThread(Move_control), NULL);

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
		//ANO_sent_data(mot1`		or_data_0->speed_rpm, set_speed_0,(int16)motor_pid_0.Pout,(int16)gein, (int16)bss,(int16)motor_pid_0.Iout, (int16)motor_pid_0.Ki,(int16)motor_pid_0.Kp ,(int16)motor_pid_0.Pout ,(int16)motor_pid_0.Dout);
		//ANO_sent_data(motor_data_0->speed_rpm, set_speed_0,(int16)kpdata,(int16)kidata, (int16)kddata,(int16)outdata, 0,0 ,0,0);
		//ANO_sent_data(motor_data_0->speed_rpm,(int16)set_speed_0, (int16)motor_pid_0.out,(int16)motor_pid_0.Pout, (int16)motor_pid_0.Iout,(int16)motor_pid_0.Dout, (int16)motor_pid_0.error[0],0 ,0,0);
		//ANO_sent_data((int16)error_tof_y,(int16)heading_deg, (int16)rof_pid.out,(int16)rof_pid.Pout, (int16)rof_pid.Iout,(int16)rof_pid.Dout,(int16)TOF1 ,(int16)TOF4,0,0);
		//ANO_sent_data((int16)TOF1,(int16)vw, (int16)turning_angle,(int16)comp_angle, (int16)rolling_flag,(int16)vx,(int16)error_tof_x ,0,0,0);
			jetson_data[0] = uart7Rx[1];
			jetson_data[1] = uart7Rx[2];
			if(jetson_data[0] == 'J' && jetson_data[1] == 'K'){
				jeston_flag = 1;
			}else if(jetson_data[0] == 'S' && jetson_data[1] == 'T'){
				jeston_flag = 2;
			}else if(jetson_data[0] == 'R' && jetson_data[1] == 'G'){
				jeston_flag = 3;
			}else if(jetson_data[0] == 'C' && jetson_data[1] == 'D'){
				jeston_flag = 4;
				test_flag = 1;
			}else if(jetson_data[0] == 'H' && jetson_data[1] == 'J'){
				jeston_flag = 5;
			}
			if(CR_flag == 1){
				if(crmm>0){
					HAL_UART_Transmit(&huart7, (uint8_t *)"AFMB", strlen("AFMB"), 999);
					crmm--;
				}else{
					CR_flag = 0;
					crmm = 3;
				}
			}
			if(last_jetson_data[0]!=jetson_data[0]&& last_jetson_data[1]!=jetson_data[1]){
				last_jetson_data[0] = jetson_data[0];
				last_jetson_data[1] = jetson_data[1];
				if(jeston_flag != 1){
					HAL_UART_Transmit(&huart7, (uint8_t *)"AJKB", strlen("AJKB"), 999);
				}
			}
    osDelay(2);
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
void PID_Control_Function(void const * argument)
{
  /* USER CODE BEGIN PID_Control_Function */
  /* Infinite loop */
  for (;;)
  {
		if(initial_flag == 1){
			if(stt > 0){
				HAL_UART_Transmit(&huart7, (uint8_t *)"AGNB", strlen("AGNB"), 999);
				stt --;
			}
			if(jeston_flag == 2){//jeston command to "Stop"
				vx = 0;
			  vy = 0;
			}
			move_solution (vx,vy,vw);
			PID_calc(&motor_pid_0, motor_data_0->speed_rpm, set_speed_0); 
			PID_calc(&motor_pid_1, motor_data_1->speed_rpm, set_speed_1); 
			PID_calc(&motor_pid_2, motor_data_2->speed_rpm, set_speed_2); 
			PID_calc(&motor_pid_3, motor_data_3->speed_rpm, set_speed_3); 
			CAN_cmd_chassis(motor_pid_0.out,motor_pid_1.out, motor_pid_2.out, motor_pid_3.out); 
			//up_move(40,1);
		}
    osDelay(2);
  }
  /* USER CODE END PID_Control_Function */
}

/* USER CODE BEGIN Header_Move_control_task */
/**
* @brief Function implementing the Move_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Move_control_task */
void Move_control_task(void const * argument)
{
  /* USER CODE BEGIN Move_control_task */
  /* Infinite loop */
  for(;;)
  {
			if(rolling_flag == 0){
				if(Fabs(TOF1-TOF4)<256){
					error_tof_y = TOF1-TOF4;
				}
				if(Fabs(TOF2-TOF3)<256){
					error_tof_x = TOF2-TOF3;
				}
				if(TOF1<= 1050&&go_to_roll == 0){
					vw = -PID_calc(&rof_pid, error_tof_y, 0);//* (1 - 2*half_move); 
				}else{
					vw = -PID_calc(&rof_pid, error_tof_x, 0);//* (1 - 2*half_move); 
				}
			}else{
				turning_angle = initial_angle + 170;
				comp_angle = (heading_deg<-90)?360 + heading_deg: heading_deg;
				if(comp_angle<turning_angle){
					vw = 600;
				}else{
					rolling_flag = 0;
				}
			}
			if(test_flag == 0){
				move_to_desk();
				if(initial_flag == 1)up_move2(level2);
			}else if(test_flag == 1){
				move_to_container();
			}
			TTL_Hex2Dec();
    osDelay(2);
  }
  /* USER CODE END Move_control_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
