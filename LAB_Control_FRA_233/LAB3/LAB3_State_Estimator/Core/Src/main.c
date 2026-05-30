/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Ultrasonic TIM_Data = { 0 };
KALMAN_1D_Params Pos_cost_data[3] = { 0 };
KALMAN_Multi_Velocity_Params Velo_cost_data[3] = { 0 };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_Header 0X0F
#define UART_Stopper 0X0A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char msg_buffer[50];
int32_t buffer;
volatile uint8_t tx_ready_flag = 1;
uint8_t TxBuffer[50];
uint8_t RxBuffer[10];
uint8_t DMAstate;
int32_t Normalvalue;
float32_t data[3];
volatile int Mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_UART_Receive_DMA(&hlpuart1, RxBuffer, 3);

	TIM_Data.state = 1;
	buffer = 100;
	DMAstate = 1;
	Normalvalue = 100;
	Mode = 0;

	KALMAN_1D_Init(&Pos_cost_data[0], 0.001f, 0.75f); //target process measure
	KALMAN_1D_Init(&Pos_cost_data[1], 0.01f, 0.75f); //target process measure
	KALMAN_1D_Init(&Pos_cost_data[2], 0.1f, 0.75f); //target process measure
	KALMAN_Multi_Velocity_Init(&Velo_cost_data[0], 1000000.0f, 0.75f);
	KALMAN_Multi_Velocity_Init(&Velo_cost_data[1], 1100000.0f, 0.75f);
	KALMAN_Multi_Velocity_Init(&Velo_cost_data[2], 1200000.0f, 0.75f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void delay() {
	{
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		while (__HAL_TIM_GET_COUNTER (&htim2) < 10)
			;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Check if the interrupt was triggered by Timer 2
	static int debugging = 0;
	debugging++;
	if (htim->Instance == TIM3) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		delay();
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	}
	if (htim->Instance == TIM4) { // 1. Compute Kalman Filters
		if (Mode == 1) {
			for (int i = 0; i < 3; i++) {
				KALMAN_1D_Compute(&Pos_cost_data[i], TIM_Data.Distance);
				data[i] = Pos_cost_data[i].x;
			}
		} else if (Mode == 2) {
			for (int i = 0; i < 3; i++) {
				KALMAN_Multi_Velocity_Compute(&Velo_cost_data[i],
						(float32_t) TIM_Data.Distance);
				data[i] = Velo_cost_data[i].X[0];
				data[0] = debugging; //debugg
			}
		} else {
			for (int i = 0; i < 3; i++) {
				data[i] = 0;
			}
		}
		uint64_t data_b[3];
		for (int i = 0; i < 3; i++) {
			double temp_d = (double) data[i];
			// This captures the raw IEEE-754 bit pattern
			data_b[i] = *(uint64_t*) &temp_d;
		}
		double dist_d = (double) TIM_Data.Distance;
		// 3. Extract raw bytes uint64_t
		uint64_t dist_b;
		dist_b = *(uint64_t*) &dist_d;
		// 4. Pack the TxBuffer (New Mapping)
		TxBuffer[0] = UART_Header; // Index 0 // Velocity 1 (Bytes 1-8)
		for (int i = 0; i < 8; i++) {
			TxBuffer[1 + i] = (uint8_t) (data_b[0] >> (8 * i));
		} // Velocity 2 (Bytes 9-16)
		for (int i = 0; i < 8; i++) {
			TxBuffer[9 + i] = (uint8_t) (data_b[1] >> (8 * i));
		} // Velocity 3 (Bytes 17-24)
		for (int i = 0; i < 8; i++) {
			TxBuffer[17 + i] = (uint8_t) (data_b[2] >> (8 * i));
		} // Distance (Bytes 25-32)
		for (int i = 0; i < 8; i++) {
			TxBuffer[25 + i] = (uint8_t) (dist_b >> (8 * i));

		}
		TxBuffer[33] = UART_Stopper; // Index 33
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // 5. Transmit 34 bytes total // (Header + 4x8 bytes + Stopper = 34)
		HAL_UART_Transmit_DMA(&hlpuart1, TxBuffer, 34);
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
// 1. Check if the interrupt came from Timer 1
if (htim->Instance == TIM1) {
	if (TIM_Data.state == 1) {
		TIM_Data.Rising = HAL_TIM_ReadCapturedValue(&htim1,
		TIM_CHANNEL_1);
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1,
				TIM_INPUTCHANNELPOLARITY_FALLING);
		TIM_Data.state = 2;
	} else if (TIM_Data.state == 2) {
		TIM_Data.Falling = HAL_TIM_ReadCapturedValue(&htim1,
		TIM_CHANNEL_1);
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1,
				TIM_INPUTCHANNELPOLARITY_RISING);
		TIM_Data.state = 1;
		TIM_Data.Diff = TIM_Data.Falling - TIM_Data.Rising;
		TIM_Data.Distance = (float) TIM_Data.Diff / 58.0f;
		__HAL_TIM_SET_COUNTER(&htim1, 0);

	}
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

if (huart == &hlpuart1) {
	Mode = RxBuffer[1];
}
}
//
//	void UARTDMAConfig(){
//		HAL_UART_Receive_DMA(&hlpart1, RxBuffer ,10);
//	}
//
//	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//		if(huart == &hlpuart1){
//
//		}
//	}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
__disable_irq();
while (1) {
}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
