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
KALMAN_Multi_Model_Params Model_data[3] = { 0 };
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
uint8_t TxBuffer[120];
uint8_t RxBuffer[50];
uint8_t DMAstate;
int32_t Normalvalue;
float32_t data_pos[3];
float32_t data_velo[3];
float32_t data_kalman_pos[3];
float32_t data_kalman_velo[3];
float32_t Model_buffer[4];

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
	HAL_UART_Receive_DMA(&hlpuart1, RxBuffer, 35);

	TIM_Data.state = 1;
	buffer = 100;
	DMAstate = 1;
	Normalvalue = 100;
	Mode = 0;

	KALMAN_1D_Init(&Pos_cost_data[0], 0.0000000075f, 0.0075f); //target process measure
	KALMAN_1D_Init(&Pos_cost_data[1], 0.0000075f, 0.0075f); //target process measure
	KALMAN_1D_Init(&Pos_cost_data[2], 0.0075f, 0.0075f); //target process measure
	KALMAN_Multi_Velocity_Init(&Velo_cost_data[0], 750000.0f, 0.0075f);
	KALMAN_Multi_Velocity_Init(&Velo_cost_data[1], 750000.0f, 0.0075f);
	KALMAN_Multi_Velocity_Init(&Velo_cost_data[2], 750000.0f, 0.0075f);
  KALMAN_Multi_Model_Init(&Model_data[0], 7.5f, 0.0075f, 0.0f, 0.0f, 0.0f, 0.0f);
  KALMAN_Multi_Model_Init(&Model_data[1], 75.0f, 0.0075f, 0.0f, 0.0f, 0.0f, 0.0f);
  KALMAN_Multi_Model_Init(&Model_data[2], 750.0f, 0.0075f, 0.0f, 0.0f, 0.0f, 0.0f);
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

// Define this in main.c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the interrupt was triggered by PA5
  if(GPIO_Pin == GPIO_PIN_13)
  {
  KALMAN_Multi_Model_Init(&Model_data[0], Model_data[0].Process_Noise, Model_data[0].R[0], Model_buffer[0], Model_buffer[1], Model_buffer[2], Model_buffer[3]);
  KALMAN_Multi_Model_Init(&Model_data[1], Model_data[1].Process_Noise, Model_data[1].R[0], Model_buffer[0], Model_buffer[1], Model_buffer[2], Model_buffer[3]);
  KALMAN_Multi_Model_Init(&Model_data[2], Model_data[2].Process_Noise, Model_data[2].R[0], Model_buffer[0], Model_buffer[1], Model_buffer[2], Model_buffer[3]);
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Check if the interrupt was triggered by Timer 2
	if (htim->Instance == TIM3) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		delay();
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	}
	if (htim->Instance == TIM4) { // 1. Compute Kalman Filters
		switch (Mode)
    {
      int i;
    case 1:
      for(i = 0 ; i<3 ; i++){
        KALMAN_1D_Compute(&Pos_cost_data[i],TIM_Data.Distance);
        data_pos[i] = Pos_cost_data[i].x;
        data_velo[i] = Pos_cost_data[i].K;
      }
      break;
    case 2:
      for(i = 0 ; i<3 ; i++){
        KALMAN_Multi_Velocity_Compute(&Velo_cost_data[i], TIM_Data.Distance);
        data_pos[i] = Velo_cost_data[i].X[0];
        data_velo[i] = Velo_cost_data[i].X[1];
      }
      break;
    case 3:
    for(i = 0 ; i<3 ; i++){
        KALMAN_Multi_Model_Compute(&Model_data[i], TIM_Data.Distance);
        data_pos[i] = Model_data[i].X[0];
        data_velo[i] = Model_data[i].X[1];
    }
      break;
    default:
    for(i = 0 ; i<3 ; i++){
        data_pos[i] = 0;
        data_velo[i] = 0;
    }
      break;
    }

 // --- DATA CONVERSION & UART TRANSMISSION ---
    
    double pos_double[3];
    double velo_double[3];
    double Kalman_pos_double[3];
    double Kalman_velo_double[3];

    // 1. Cast float32 data to double (64-bit)
    for(int i = 0; i < 3; i++) {
      pos_double[i]  = (double)data_pos[i];
      velo_double[i] = (double)data_velo[i];
      Kalman_pos_double[i] = (double)Kalman_pos_double[i];
      Kalman_velo_double[i] = (double)Kalman_velo_double[i];

    }
    for(int i = 0; i < 3; i++) {
      memcpy(&TxBuffer[(i*32)+1], &pos_double[i], sizeof(pos_double[i]));
      memcpy(&TxBuffer[(i*32)+9], &velo_double[i], sizeof(velo_double[i]));
      memcpy(&TxBuffer[(i*32)+17], &Kalman_pos_double[i], sizeof(double));
      memcpy(&TxBuffer[(i*32)+25], &Kalman_velo_double[i], sizeof(double));
    }
    double Distance_buffer = (double)TIM_Data.Distance;
      memcpy(&TxBuffer[97], &Distance_buffer, sizeof(Distance_buffer));
    TxBuffer[0] = UART_Header;
    TxBuffer[105] = UART_Stopper;


    
		HAL_UART_Transmit_DMA(&hlpuart1, TxBuffer, 106);
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
        
        // 1. Process the 4 double variables (Bytes 0 through 31)
       for (int i = 0; i < 4; i++) {
           uint64_t raw_bits = 0;

           // Shift 8 bytes into a 64-bit integer (Assuming Little-Endian payload)
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 1] << 0);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 2] << 8);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 3] << 16);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 4] << 24);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 5] << 32);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 6] << 40);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 7] << 48);
           raw_bits |= ((uint64_t)RxBuffer[(i * 8) + 8] << 56);

           // Type-pun the raw 64-bit integer into a double safely
           double temp_double;
           memcpy(&temp_double, &raw_bits, sizeof(double));

           // Cast the 64-bit double down to 32-bit float and store it
           Model_buffer[i] = (float32_t)temp_double;
       }

       // 2. Process the final int8_t Mode variable (Byte index 32)
       if(RxBuffer[0] == 14) {
           Mode = RxBuffer[33];
       }
        
        // Remember to re-arm the interrupt to listen for the next payload
//        HAL_UART_Receive_IT(&hlpuart1, RxBuffer, 34);
    }
}

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
