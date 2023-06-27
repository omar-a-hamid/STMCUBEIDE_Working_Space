/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Lidar.h"
#include "UltraSonic.h"
#include "string.h"
#include "gps_driver.h"
#include"LCD.h"
#include <string.h>
#include <stdio.h>

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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for LidarREAD */
osThreadId_t LidarREADHandle;
const osThreadAttr_t LidarREAD_attributes = {
		.name = "LidarREAD",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TakeAction_Lida */
osThreadId_t TakeAction_LidaHandle;
const osThreadAttr_t TakeAction_Lida_attributes = {
		.name = "TakeAction_Lida",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Ultrasonic_Read */
osThreadId_t Ultrasonic_ReadHandle;
const osThreadAttr_t Ultrasonic_Read_attributes = {
		.name = "Ultrasonic_Read",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS */
osThreadId_t GPSHandle;
const osThreadAttr_t GPS_attributes = {
		.name = "GPS",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Send_Data */
osThreadId_t Send_DataHandle;
const osThreadAttr_t Send_Data_attributes = {
		.name = "Send_Data",
		.stack_size = 300 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Rec_Data */
osThreadId_t Rec_DataHandle;
const osThreadAttr_t Rec_Data_attributes = {
		.name = "Rec_Data",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
#define RxBuf_SIZE 800

uint8_t RxBuf[RxBuf_SIZE];
uint8_t RMC[80];
uint8_t GGA[80];
uint8_t msgindex=0;
uint8_t msgindexRMC=0;

GGA_STRUCT GGA_DATA;

RMC_STRUCT RMC_DATA;

int Flag=0;
char str_GGA[50];
char str_RMC[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartLidarREAD(void *argument);
void Start_TakeAction_Lidar(void *argument);
void Start_Ultrasonic_Read(void *argument);
void Start_GPS(void *argument);
void StartTask06(void *argument);
void Start_Rec_Transmit(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*****extern variable****/
extern uint16_t Global_u16LidarDistance;
extern uint32_t Rear_Distance;
extern uint32_t Left_Distance;
extern uint32_t Right_Distance;
char test_data[400];
int Vehicle_ID = 1;
int Routing_command=0;
float Destination_Longitude=31.3488351;
float Destination_Latitude=30.0591282;
int Check_Front_Obs=0;

char rx_data[10];
int Flag_Rec =0;
int Flag_obstacle=0;
int Flag_Drive=0;
uint8_t blink_color = 0;


/*buffer to receive data */
uint8_t g_usart1_rx_buf[USART_BUF_SIZE] = { 0 };
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		Flag=1;
		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
	if (huart->Instance == USART3)
	{
		Flag_Rec=1;
		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) rx_data, sizeof(rx_data));
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

	}
}



/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_data, sizeof(rx_data));
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	LCD_init(); /* Initialize the LCD */
	LCD_displayStringRowColumn(0,0,"STM LCD Driver");
	LCD_displayStringRowColumn(1,3,"GRAD PROJECT");
	HAL_Delay(400); /* wait four seconds */

	LCD_clearScreen(); /* clear the LCD display */
	LCD_displayString(" ");
	LCD_displayStringRowColumn(1,3,"4-Bit Mode");
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

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
	/* creation of LidarREAD */
	LidarREADHandle = osThreadNew(StartLidarREAD, NULL, &LidarREAD_attributes);

	/* creation of TakeAction_Lida */
	TakeAction_LidaHandle = osThreadNew(Start_TakeAction_Lidar, NULL, &TakeAction_Lida_attributes);

	/* creation of Ultrasonic_Read */
	Ultrasonic_ReadHandle = osThreadNew(Start_Ultrasonic_Read, NULL, &Ultrasonic_Read_attributes);

	/* creation of GPS */
	GPSHandle = osThreadNew(Start_GPS, NULL, &GPS_attributes);

	/* creation of Send_Data */
	Send_DataHandle = osThreadNew(StartTask06, NULL, &Send_Data_attributes);

	/* creation of Rec_Data */
	Rec_DataHandle = osThreadNew(Start_Rec_Transmit, NULL, &Rec_Data_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 168-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xffff-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
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

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
			|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC0 PC1 PC2 PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PE10 PE12 PE14 */
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
			|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLidarREAD */
/**
 * @brief  Function implementing the LidarREAD thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLidarREAD */
void StartLidarREAD(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		HAL_UART_Receive_DMA(&huart1, g_usart1_rx_buf, USART_BUF_SIZE);

		/* enable uart IDLE  interrupt */
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		if(Global_u16LidarDistance <=100)
		{
			Flag_obstacle=1;
			Flag_Drive=0;
		}
		if(Global_u16LidarDistance < 1200)
		{
			Check_Front_Obs=1;
		}
		else if(Global_u16LidarDistance == 65535)
		{
			Check_Front_Obs=0;
		}
		osDelay(800);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_TakeAction_Lidar */
/**
 * @brief Function implementing the TakeAction_Lida thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_TakeAction_Lidar */
void Start_TakeAction_Lidar(void *argument)
{
	/* USER CODE BEGIN Start_TakeAction_Lidar */
	/* Infinite loop */
	for(;;)
	{
		if (Flag_obstacle == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,SET);
			LCD_clearScreen(); /* clear the LCD display */
			LCD_displayString(" ");
			LCD_displayStringRowColumn(0,0,"Applying Brakes");
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,RESET);
			if(Global_u16LidarDistance >=101)
			{
				Flag_Drive=1;
			}
			Flag_obstacle=0;
			osDelay(200);

		}
		else if(Flag_Drive==1) {
			LCD_clearScreen(); /* clear the LCD display */
			LCD_displayString(" ");
			LCD_displayStringRowColumn(0,0,"Drive Mode");
			Flag_Drive=0;
			osDelay(20);
		}
	}
	/* USER CODE END Start_TakeAction_Lidar */
}

/* USER CODE BEGIN Header_Start_Ultrasonic_Read */
/**
 * @brief Function implementing the Ultrasonic_Read thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Ultrasonic_Read */
void Start_Ultrasonic_Read(void *argument)
{
	/* USER CODE BEGIN Start_Ultrasonic_Read */
	/* Infinite loop */
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	//HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);


	for(;;)
	{
		UltraSonic_Read_Left();
		osDelay(400);

		UltraSonic_Read_Right();
		osDelay(400);
	}
	/* USER CODE END Start_Ultrasonic_Read */
}

/* USER CODE BEGIN Header_Start_GPS */
/**
 * @brief Function implementing the GPS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_GPS */
void Start_GPS(void *argument)
{
	/* USER CODE BEGIN Start_GPS */
	/* Infinite loop */
	for(;;)
	{
		if (Flag==1)
		{
			//	if(Flag == 1){
			char *ptr = strstr((char *)RxBuf,"GPGGA");
			char *ptr1 = strstr((char *)RxBuf,"GPRMC");


			if(*ptr == 'G' )
			{
				if(*(ptr+1) == 'P' )

				{
					if(*(ptr+2) == 'G' )
					{
						if(*(ptr+3) == 'G' )
						{
							if(*(ptr+4) == 'A' )
							{
								while(1)
								{
									GGA[msgindex]=*ptr;
									msgindex++;
									*ptr =*(ptr+msgindex);
									if (*ptr=='*')
									{
										GGA[msgindex]='\n';
										msgindex++;
										GGA[msgindex]='\0';
										msgindex=0;
										break;
									}
								}

							}
						}
					}
				}
			}


			if(*ptr1 == 'G' )
			{
				if(*(ptr1+1) == 'P' )

				{
					if(*(ptr1+2) == 'R' )
					{
						if(*(ptr1+3) == 'M' )
						{
							if(*(ptr1+4) == 'C' )
							{
								while(1)
								{
									RMC[msgindexRMC]=*ptr1;
									msgindexRMC++;
									*ptr1 =*(ptr1+msgindexRMC);
									if (*ptr1=='*')
									{
										RMC[msgindexRMC]='\n';
										msgindexRMC++;
										RMC[msgindexRMC]='\0';
										msgindexRMC=0;
										break;
									}
								}

							}
						}
					}
				}
			}
			RMC_decode(RMC, &RMC_DATA);
			GGA_decode(GGA, &GGA_DATA);
			Flag=0;
			osDelay(100);
		}	}
	/* USER CODE END Start_GPS */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
 * @brief Function implementing the Send_Data thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
	/* USER CODE BEGIN StartTask06 */
	/* Infinite loop */
	for(;;)
	{
		char lat[30];
		char lon[30];
		char D_lat[30];
		char D_lon[30];

		gcvt(GGA_DATA.position.latitude, 10, lat);
		gcvt(GGA_DATA.position.logitude, 10, lon);
		gcvt(Destination_Latitude, 10, D_lat);
		gcvt(Destination_Longitude, 10, D_lon);

		/*	int len = snprintf(test_data, sizeof(test_data), "%d-%d-%d %d:%d:%d,%d,%s,%s,%d,%d,%d,%lu,%lu,%lu,%s,%s"
				,
				RMC_DATA.date.day, RMC_DATA.date.month, RMC_DATA.date.year,
				GGA_DATA.time.hour, GGA_DATA.time.minuit, GGA_DATA.time.second, Vehicle_ID,
				lon, lat,(uint16_t)RMC_DATA.speed_over_gnd, Check_Front_Obs, Global_u16LidarDistance,
				Left_Distance, Right_Distance,Rear_Distance, D_lon, D_lat);
		 */


		/*int len = snprintf(test_data, sizeof(test_data), "{\n"
				"\"dateandtime\": \"%d-%d-%d %d:%d:%d\" ,\n"
				"\"vehicle id\": %d,\n"
				"\"current lon\": %s,\n"
				"\"current lat\": %s,\n"
				"\"spdK/m\": %d,\n"
				"\"obsatcle flag\": %d,\n"
				"\"obstacle distance\": %d,\n"
				"\"3\": %lu,\n"
				"\"4\" :%lu,\n"
				"\"5\" :%lu,\n"
				"\"destination lon\": %s,\n"
				"\"destinationn lat\": %s\n}"
				,
				RMC_DATA.date.day, RMC_DATA.date.month, RMC_DATA.date.year,
				GGA_DATA.time.hour, GGA_DATA.time.minuit, GGA_DATA.time.second, Vehicle_ID,
				lon, lat,(uint16_t)RMC_DATA.speed_over_gnd, Check_Front_Obs, Global_u16LidarDistance,
				Left_Distance, Right_Distance,Rear_Distance, D_lon, D_lat);*/
		int len = snprintf(test_data, sizeof(test_data), "{\n"
				"\"dateandtime\": \"2022-12-07 08:48:00\" ,\n"
				"\"id\": \"%d\",\n"
				"\"r_cmd\": %d,\n"
				"\"clon\": 31.338135,\n"
				"\"clat\": 30.056193,\n"
				"\"spdK/m\": %d,\n"
				"\"1\": %d,\n"//obs flag
				"\"2\": %d,\n"//obs distance
				"\"3\": %lu,\n" // left distance
				"\"4\" :%lu,\n"//right
				"\"5\": %s,\n" //dest lon
				"\"6\": %s\n}" // dest lat

				/*RMC_DATA.date.year, RMC_DATA.date.month, RMC_DATA.date.day,
				GGA_DATA.time.hour, GGA_DATA.time.minuit, GGA_DATA.time.second*/, Vehicle_ID, Routing_command,
				/*lon, lat,*/(uint16_t)RMC_DATA.speed_over_gnd, Check_Front_Obs, Global_u16LidarDistance,
				Left_Distance, Right_Distance, D_lon, D_lat);



		/*
		 * TIME_STAMP          = "dateandtime"


		V_ID                = "vehicle id"
		CURRENT_SPEED       = "spdK/m"
		CURRENT_POS_LAT     = "current lat"
		CURRENT_POS_LON     = "current lon"

		ROUTING_CMD         = "routing command"
		DISTINATION_POS_LAT = "destinationn lat"
		DISTINATION_POS_LON = "destination lon"

		OBSTACLE_FLAG       = "obsatcle flag"
		OBSTACLE_SPEED      = "obstacle speed"
		OBSTACLE_distance   = "obstacle distance"
		 */
		if (len < 0 || len >= sizeof(test_data)) {
			// handle error
		} else {
			HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)test_data, sizeof(test_data));
			//HAL_Delay(100);
			if (status != HAL_OK) {
				// handle error
			} else {
				printf("Transmitted %d bytes of data: %s\n", sizeof(test_data), test_data);
			}
		}

		osDelay(1000);
	}
	/* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_Start_Rec_Transmit */
/**
 * @brief Function implementing the Rec_Data thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Rec_Transmit */
void Start_Rec_Transmit(void *argument)
{
	/* USER CODE BEGIN Start_Rec_Transmit */
	/* Infinite loop */
	for(;;)
	{

		if (Flag_Rec == 1)
		{
			rx_data[strcspn(rx_data, "\r\n")] = '\0';
			rx_data[strcspn(rx_data, "\r\n")] = '\0';
			if (strcmp(rx_data, "Right") == 0)
			{
				blink_color = 1; // Green
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, RESET);
				memset((rx_data), '\0', strlen(rx_data));
				LCD_clearScreen(); /* clear the LCD display */
				LCD_displayString(" ");
				LCD_displayStringRowColumn(1,3,"Turn Right");
			}
			else if( strcmp(rx_data, "Left") == 0)
			{
				blink_color = 2; // Yellow
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, RESET);
				memset((rx_data), '\0', strlen(rx_data));

				LCD_clearScreen(); /* clear the LCD display */
				LCD_displayString(" ");
				LCD_displayStringRowColumn(1,3,"Turn Left");
			} else if (strcmp(rx_data, "Straight") == 0)
			{
				blink_color = 3; // Blue
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, RESET);
				memset((rx_data), '\0', strlen(rx_data));

				LCD_clearScreen(); /* clear the LCD display */
				LCD_displayString(" ");
				LCD_displayStringRowColumn(1,3,"Go Straight");

			} else if (strcmp(rx_data, "Warning") == 0)
			{
				blink_color = 4; // Red
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, SET);
				memset((rx_data), '\0', strlen(rx_data));

				LCD_clearScreen(); /* clear the LCD display */
				LCD_displayString(" ");
				LCD_displayStringRowColumn(1,3,"Warning!!");
			}

			Flag_Rec=0;
		}	}
	/* USER CODE END Start_Rec_Transmit */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
