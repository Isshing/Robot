/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "struct_typedef.h"
#include <string.h>
#include <stdio.h>
#include "IMU.h"

#define HEADER 0xAA							/* ��ʼ�� */
#define device_address 0x00     /* �豸��ַ */
#define chunk_offset 0x00       /* ƫ�Ƶ�ַ���� */
#define PACK_GET_DISTANCE 0x02 	/* ��ȡ������������ */
#define PACK_RESET_SYSTEM 0x0D 	/* ��λ���� */
#define PACK_STOP 0x0F 				  /* ֹͣ�������ݴ������� */
#define PACK_ACK 0x10           /* Ӧ�������� */
#define PACK_VERSION 0x14       /* ��ȡ��������Ϣ���� */
uint8_t uart7Rx[4];         
uint16_t uart7RxLength;

uint8_t uart8Rx[32];          
uint16_t uart8RxLength;

//uint8_t uart6Tx[32];          
//uint16_t uart6TxLength;

//uint8_t uart6Rx[32];          
//uint16_t uart6RxLength;
/* USER CODE END 0 */

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart8_rx;

/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}
/* UART8 init function */
void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 230400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART7 DMA Init */
    /* UART7_RX Init */
    hdma_uart7_rx.Instance = DMA1_Stream3;
    hdma_uart7_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode = DMA_NORMAL;
    hdma_uart7_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart7_rx);

  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* UART8 clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART8 DMA Init */
    /* UART8_RX Init */
    hdma_uart8_rx.Instance = DMA1_Stream6;
    hdma_uart8_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart8_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart8_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart8_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart8_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart8_rx);

  /* USER CODE BEGIN UART8_MspInit 1 */

  /* USER CODE END UART8_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD6     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8|GPIO_PIN_7);

    /* UART7 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspDeInit 0 */

  /* USER CODE END UART8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART8_CLK_DISABLE();

    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1|GPIO_PIN_0);

    /* UART8 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
  /* USER CODE BEGIN UART8_MspDeInit 1 */

  /* USER CODE END UART8_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD6     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6|GPIO_PIN_5);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
char jetson_data[2];
void USR_UartInit(void)
{

		uart7RxLength = 0;
		uart8RxLength = 0;
//		uart6TxLength = 0;
//		uart6RxLength = 0;
		HAL_UART_Receive_DMA(&huart7, uart7Rx, 4);
		HAL_UART_Receive_DMA(&huart8, uart8Rx, 32);			
		//HAL_UART_Receive_DMA(&huart6, uart6Rx, 32);	

		__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE); 
		__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE); 
	  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 
	  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); 
}



void Jetson_read(unsigned char *data){
		for(int j=0;j<16;j++)
		{
			if(data[j] == 'A' && data[j+3] == 'B'){
				jetson_data[0] = data[j+1];
				jetson_data[1] = data[j+2];
			}
		}
}
unsigned int jeston_flag = 0;
char * call_jeston = "OK";
char buffer6[32];
uint8_t count_imu = 0;
uint8_t last_rsnum = 0;
uint8_t rsimu_flag = 0, rsacc_flag = 0;
uint8_t RxByte;
uint8_t Jetson_RxByte;
extern float vx,vy,vw ;
extern int st;
extern int test_flag;
uint8_t jetson_rx[4];
int count_jetson = 0;
uint8_t Uart3_Receive_buf[1];          //����5�����ж����ݴ�ŵĻ�����
LidarPointTypedef Pack_Data[12];/* �״���յ����ݴ������������֮�� */
LidarPointTypedef Pack_sum;     /* ���������� */
uint16_t receive_cnt;
uint8_t confidence;
uint16_t distance,noise,reftof;
uint32_t peak,intg;
void data_process(void)/*���ݴ����������һ֮֡��ɽ������ݴ���*/
{
		/* ������� */
		u8 i;
		static u16 count = 0;
		LidarPointTypedef Pack_sum;
		for(i=0;i<12;i++)									/* 12����ȡƽ�� */
		{
				if(Pack_Data[i].distance != 0)  /* ȥ��0�ĵ� */
				{
						count++;
						Pack_sum.distance += Pack_Data[i].distance;
						Pack_sum.noise += Pack_Data[i].noise;
						Pack_sum.peak += Pack_Data[i].peak;
						Pack_sum.confidence += Pack_Data[i].confidence;
						Pack_sum.intg += Pack_Data[i].intg;
						Pack_sum.reftof += Pack_Data[i].reftof;
				}
		}
		if(count !=0)
		{

					distance = Pack_sum.distance/count;
					noise = Pack_sum.noise/count;
					peak = Pack_sum.peak/count;
					confidence = Pack_sum.confidence/count;
					intg = Pack_sum.intg/count;
					reftof = Pack_sum.reftof/count;
					Pack_sum.distance = 0;
					Pack_sum.noise = 0;
					Pack_sum.peak = 0;
					Pack_sum.confidence = 0;
					Pack_sum.intg = 0;
					Pack_sum.reftof = 0;
					count = 0;
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		static uint8_t state = 0;			//״̬λ	
		static uint8_t crc = 0;				//У���
		static uint8_t cnt = 0;				//����һ֡12����ļ���
		static uint8_t PACK_FLAG = 0;  //�����־λ
		static uint8_t data_len  = 0;  //���ݳ���
		static uint32_t timestamp = 0; //ʱ���
		static uint8_t state_flag = 1; //ת�����ݽ��ձ�־λ
		uint8_t temp_data;
		if(huart == &huart7)
		{
			//AD_main_Filter(TOF_distance7,TOF_distance8);
//			jetson_rx[count_jetson] = Jetson_RxByte;
//			if(jetson_rx[0] == 'A'){
//				count_jetson ++ ;
//			}else{
//				count_jetson = 0;
//			}
//			if(jetson_rx[0] == 'A'&& jetson_rx[3] == 'B'){
//				jetson_data[0] = jetson_rx[1];
//				jetson_data[1] = jetson_rx[2];
//				memset(jetson_rx, 0, sizeof(jetson_rx));
//				count_jetson = 0;
//			}

			HAL_UART_Receive_DMA(&huart7, uart7Rx, 4);
		}
		else if(huart == &huart8)
		{

			HAL_UART_Receive_DMA(&huart8, uart8Rx, 32);
		}
		else if(huart == &huart6)
		{
			temp_data=Uart3_Receive_buf[0];	
			if(state< 4) 																					 /* ��ʼ����֤ ǰ4�����ݾ�Ϊ0xAA */
				{                                          
						if(temp_data == HEADER) state ++;
						else state = 0;
				}
				else if(state<10&&state>3)
				{
						switch(state)
						{
								case 4:   
									if(temp_data == device_address)              /* �豸��ַ��֤ */
									{							
													state ++;
													crc = crc + temp_data;									
													break;        
									} 
									else state = 0,crc = 0;
								case 5:   
									if(temp_data == PACK_GET_DISTANCE)					 /* ��ȡ������������ */
									{  
													PACK_FLAG = PACK_GET_DISTANCE;
													state ++;	
													crc = crc + temp_data;	
													break;									
									}		 

									else if(temp_data == PACK_RESET_SYSTEM) 		 /* ��λ���� */
									{
													PACK_FLAG = PACK_RESET_SYSTEM;
													state ++; 
													crc = crc + temp_data;	
													break;	
									}
									else if(temp_data == PACK_STOP)							 /* ֹͣ�������ݴ������� */
									{ 
													PACK_FLAG = PACK_STOP;
													state ++; 
													crc = crc + temp_data;	
													break;
									}
									else if(temp_data == PACK_ACK)							 /* Ӧ�������� */
									{  
													PACK_FLAG = PACK_ACK;
													state ++;
													crc = crc + temp_data;	
													break;
									}			 				 
									else if(temp_data == PACK_VERSION)					 /* ��ȡ��������Ϣ���� */
									{ 
													PACK_FLAG = PACK_VERSION,
													state ++,
													crc = crc + temp_data;	   	     
													break;
									}
									else state = 0,crc = 0;
								case 6: if(temp_data == chunk_offset)          /* ƫ�Ƶ�ַ */
												{  
													state ++;
													crc = crc + temp_data;
													break; 	  
												}	
												else state = 0,crc = 0;
								case 7: if(temp_data == chunk_offset)
												{  
													state ++;
													crc = crc + temp_data;
													break;
												}
												else state = 0,crc = 0;
								case 8: 
										data_len = (u16)temp_data;								 /* ���ݳ��ȵͰ�λ */
										state ++; 
										crc = crc + temp_data;
										break;																			 
								case 9: 
										data_len = data_len + ((u16)temp_data<<8); 			 /* ���ݳ��ȸ߰�λ */
										state ++;
										crc = crc + temp_data;
										break; 
								default: break;
						}
				}
				else if(state == 10 ) state_flag = 0;                    /*��switch������ʱstateΪ10����temp_data��Ϊ���볤�ȸ߰�λ���ݣ�������һ���ж�*/
				if(PACK_FLAG == PACK_GET_DISTANCE&&state_flag == 0)      /* ��ȡһ֡���ݲ�У�� */
				{
						if(state>9)
						{
								if(state<190)
								{
										static uint8_t state_num;
										state_num = (state-10)%15;
										switch(state_num)
										{
												case 0: 
													Pack_Data[cnt].distance = (uint16_t)temp_data ;				 /* �������ݵͰ�λ */
													crc = crc + temp_data;
													state++;
													break;        
												case 1: 
													Pack_Data[cnt].distance = ((u16)temp_data<<8) + Pack_Data[cnt].distance;	 /* �������� */
													crc = crc + temp_data;
													state++;
													break; 
												case 2:
													Pack_Data[cnt].noise = (u16)temp_data;				 /* ���������Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 3:
													Pack_Data[cnt].noise = ((u16)temp_data<<8) + Pack_Data[cnt].noise;				 /* �������� */
													crc = crc + temp_data;
													state++;
													break; 
												case 4:
													Pack_Data[cnt].peak = (u32)temp_data;				 										 /* ����ǿ����Ϣ�Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 5:
													Pack_Data[cnt].peak = ((u32)temp_data<<8) + Pack_Data[cnt].peak;
													crc = crc + temp_data;
													state++;
													break; 
												case 6:
													Pack_Data[cnt].peak = ((u32)temp_data<<16) + Pack_Data[cnt].peak;	
													crc = crc + temp_data;
													state++;
													break; 
												case 7:
													Pack_Data[cnt].peak = ((u32)temp_data<<24) + Pack_Data[cnt].peak;				    /* ����ǿ����Ϣ */
													crc = crc + temp_data;
													state++;
													break; 
												case 8:
													Pack_Data[cnt].confidence = temp_data;				 /* ���Ŷ� */
													crc = crc + temp_data;
													state++;
													break; 
												case 9:
													Pack_Data[cnt].intg = (u32)temp_data;															/* ���ִ����Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 10:
													Pack_Data[cnt].intg = ((u32)temp_data<<8) + Pack_Data[cnt].intg;
													crc = crc + temp_data;
													state++;
													break; 
												case 11:
													Pack_Data[cnt].intg = ((u32)temp_data<<16) + Pack_Data[cnt].intg;
													crc = crc + temp_data;
													state++;
													break; 
												case 12:
													Pack_Data[cnt].intg = ((u32)temp_data<<24) + Pack_Data[cnt].intg;				  	 /* ���ִ��� */
													crc = crc + temp_data;
													state++;
													break; 
												case 13:
													Pack_Data[cnt].reftof = (int16_t)temp_data;				 								 /* �¶ȱ���ֵ�Ͱ�λ */
													crc = crc + temp_data;
													state++;
													break; 
												case 14:
													Pack_Data[cnt].reftof = ((int16_t)temp_data<<8) +Pack_Data[cnt].reftof;			/* �¶ȱ���ֵ */
													crc = crc + temp_data;
													state++;
													cnt++;							 /* ������һ�������� */
													break; 
												default: break;
										}
							}
										/* ʱ��� */
										if(state == 190) timestamp = temp_data,state++,crc = crc + temp_data;
										else if(state == 191) timestamp = ((u32)temp_data<<8) + timestamp,state++,crc = crc + temp_data; 
										else if(state == 192) timestamp = ((u32)temp_data<<16) + timestamp,state++,crc = crc + temp_data;
										else if(state == 193) timestamp = ((u32)temp_data<<24) + timestamp,state++,crc = crc + temp_data; 
										else if(state==194)
										{
													if(temp_data == crc)   /* У��ɹ� */
													{
															data_process();  	 /* ���ݴ����������һ֮֡��ɽ������ݴ��� */
															receive_cnt++;	 	 /* ������յ���ȷ���ݵĴ��� */
													}
													distance = Pack_Data[0].distance;
													crc = 0;
													state = 0;
													state_flag = 1;
													cnt = 0; 							 /* ��λ*/
										}
							
						}
				}
			HAL_UART_Receive_IT(&huart6,Uart3_Receive_buf,sizeof(Uart3_Receive_buf));//����5�ص�����ִ�����֮����Ҫ�ٴο��������жϵȴ���һ�ν����жϵķ���
		}
    else if(huart == &huart2) {
        Fd_data[count_imu] = RxByte;
        if (((last_rsnum == FRAME_END) && (RxByte == FRAME_HEAD)) || count_imu > 0) {
            count_imu++;
            if (count_imu == 1) { // ���������ж�Ӧ���ڽ��յ��㹻���ֽں����
                if (Fd_data[1] == TYPE_IMU) rsimu_flag = 1;
                else if (Fd_data[1] == TYPE_AHRS) rsacc_flag = 1;
            }
            if (rsimu_flag == 1 && count_imu == IMU_RS) {
                count_imu = 0;
                rsimu_flag = 0;
                rs_imutype = 1;
                if (Fd_data[IMU_RS - 1] == FRAME_END) {
                    memcpy(Fd_rsimu, Fd_data, IMU_RS);
                }
            } else if (rsacc_flag == 1 && count_imu == AHRS_RS) {
                count_imu = 0;
                rsacc_flag = 0;
                rs_ahrstype = 1;
                if (Fd_data[AHRS_RS - 1] == FRAME_END) {
                    memcpy(Fd_rsahrs, Fd_data, AHRS_RS);
                }
                memset(Fd_data, 0, sizeof(Fd_data)); // ��ս��ջ�����
            }
        } else {
            count_imu = 0; // ���ü�����
        }
        last_rsnum = RxByte;
        // ��������UART����
				
				
        HAL_UART_Receive_IT(&huart2, &RxByte, 1);
				
    }
}


#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )     /*!< uint32_t ���ݲ�� byte0  */
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )     /*!< uint32_t ���ݲ�� byte1  */
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )     /*!< uint32_t ���ݲ�� byte2  */
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )     /*!< uint32_t ���ݲ�� byte3  */

uint8 data_to_send[50];


/*������λ��ͬʱ��ʾ10������*/
void ANO_sent_data(int16 A, int16 B, int16 C, int16 D, int16 E, int16 F, int16 G, int16 H, int16 I, int16 J)
{
    uint8 i;
    uint8 checksum = 0;
    uint8 addcheck = 0;
    uint8 _cnt=0;


    data_to_send[_cnt++]=0xAA;          //֡ͷ
    data_to_send[_cnt++]=0xFF;          //Ŀ���ַ
    data_to_send[_cnt++]=0XF1;          //������
    data_to_send[_cnt++]=0x14;          //���ݳ��� 20�ֽ�
    data_to_send[_cnt++]=BYTE0(A);      //��������,С��ģʽ����λ��ǰ
    data_to_send[_cnt++]=BYTE1(A);      //��Ҫ���ֽڽ��в�֣���������ĺ궨�弴��
    data_to_send[_cnt++]=BYTE0(B);
    data_to_send[_cnt++]=BYTE1(B);
    data_to_send[_cnt++]=BYTE0(C);
    data_to_send[_cnt++]=BYTE1(C);
    data_to_send[_cnt++]=BYTE0(D);
    data_to_send[_cnt++]=BYTE1(D);
    data_to_send[_cnt++]=BYTE0(E);
    data_to_send[_cnt++]=BYTE1(E);
    data_to_send[_cnt++]=BYTE0(F);
    data_to_send[_cnt++]=BYTE1(F);
    data_to_send[_cnt++]=BYTE0(G);
    data_to_send[_cnt++]=BYTE1(G);
    data_to_send[_cnt++]=BYTE0(H);
    data_to_send[_cnt++]=BYTE1(H);
    data_to_send[_cnt++]=BYTE0(I);
    data_to_send[_cnt++]=BYTE1(I);
    data_to_send[_cnt++]=BYTE0(J);
    data_to_send[_cnt++]=BYTE1(J);

    for(i=0;i<data_to_send[3]+4;i++)
    {
        checksum+=data_to_send[i];
        addcheck+=checksum;
    }

    data_to_send[_cnt++] = checksum;      //��У��
    data_to_send[_cnt++] = addcheck;      //����У��

    HAL_UART_Transmit((UART_HandleTypeDef *)&huart6, (uint8_t *)data_to_send, (uint16_t)_cnt, (uint32_t)999);

}

/* USER CODE END 1 */
