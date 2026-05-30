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
#include "fdcan.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BaseSystem.h"
#include "kalman.h"
#include "trajectory.h"
#include "cascade.h"
#include "system_state.h"
#include "elec_cabient.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_MODE 1
#define MODE_MODBUS 0
#define MODE_MATLAB 1

//Inner Loop ---------
#define Kp_inner 0.52259f
//#define Ki_inner 0.0f
#define Ki_inner 10.2259f

#define Kd_inner 0.0f
//Outer Loop ---------
#define Kp_outer 2.0f
#define Ki_outer 0.0f
#define Kd_outer 1.0f



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- OVERALL MEMMORY ----*/
SYSTEM_STATE         sys_state;
/* ---- Hardware objects ---- */
Cascade            Controller;
PID                Inner;
PID                Outer;
KALMAN_Multi_Model_Params Kalman;
Feedforward        ff;
Encoder            encoder;
Proximity            ref_pos;
Joystick             robot_joy;
Pilot_ramp           robot_ramp;
/* ---- Trajectory objects ---- */
Traj_MinJerk_t        my_min_jerk;
Traj_MinJerk_Vlim_t   my_min_jerk_vlim;
Traj_Trapezoidal_t    my_trapezoid;
Traj_SCurveLimits_t   my_scurve;
Traj_State_t          trajectory;

/* ---- Trajectory selector (set from debugger or BaseSystem) ----
 *   0 = MinJerk (time-based)        → f32_buffer[7] = total_time
 *   1 = MinJerk_Vlim (vel-limited)  → f32_buffer[7] = v_max  (best for backlash)
 *   2 = Trapezoidal                 → f32_buffer[7] = total_time, f32_buffer[8] = accel_time
 *   3 = S-Curve (kinematic limits)  → f32_buffer[7]=v_max, [8]=a_max, [9]=j_max
 * ---------------------------------------------------------------- */
int       traj_profile  = 1;      /* which profile to use         */
float32_t traj_target   = 0.0f;   /* target position (rad)        */
float32_t traj_elapsed  = 0.0f;   /* time accumulator (seconds)   */
int       traj_running  = 0;      /* 1 = trajectory active        */

/* ---- Motor ---- */
MOTOR_PARAMS my_motor = {
   .kt  = 0.20906532f,
   .km  = 0.2316f,
   .J   = 0.107E0f,
   .B   = 0.995f,
   .R   = 0.4821323156f,
   .L   = 0.0002893301219f,
   .tau = 0.01f          /* 10 ms — safe starting tau (was 1 ms → too aggressive) */
};

//Debuf Variable
int int_buffer[12];
float32_t f32_buffer[10];
uint8_t Txbuffer[60];
float32_t vout;
float32_t Velo_debug;
int moving;
float32_t ff_friciton;
float32_t measurement_diff;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Robot_State_Process(void);
void Robot_Period_Control_Loop();
void TRAJ_Start();
Traj_State_t TRAJ_Step();
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
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM20_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
// 1. Start Hardware Timers
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_RegisterCallback(&htim20, HAL_TIM_PERIOD_ELAPSED_CB_ID, Robot_Period_Control_Loop);
  HAL_TIM_Base_Start_IT(&htim20);
  // 2. Initialize Encoder Pointers
  SYSTEM_STATE_Encoder_Init(&htim2, &htim3, &encoder);
  TRAJ_State_Init(&trajectory, &htim2);
  // 3. Setup initial state machine defaults
  sys_state.cur_state = STATE_WAITING_COMMAND;
  sys_state.trust     = Trust_Basesystem; 
  sys_state.home_pos  = 0.0f;
  
//  hmodbus.huart = &huart2;
//  hmodbus.htim = &htim16;
//  hmodbus.slaveAddress = 0x15;
//  hmodbus.RegisterSize = BASE_SYSTEM_REG_COUNT;
//  Modbus_init(&hmodbus, registerFrame);
  __HAL_TIM_SET_COUNTER(&htim3, 30000);

  /* ====================================================================
   * 4. ACTIVATE ALGORITHM INITIALIZATIONS HERE
   * ==================================================================== */
  KALMAN_Multi_Model_Init(&Kalman, 3e-8f,9.9e-5f, 8e-4f, &my_motor);
  CASCADE_Controller_Init(Kp_inner, Ki_inner, Kd_inner, &Inner, -12.0f, 12.0f, 0.0005f);  
  CASCADE_Controller_Init(Kp_outer, Ki_outer, Kd_outer, &Outer, -4.0f, 4.0f, 0.005f);
  CASCADE_Cascade_Start(&Controller, &Inner, &Outer, &ff, &my_motor, &Kalman);

  /* Pre-init all trajectory planners with safe defaults */
  TRAJ_MinJerk_Plan       (&my_min_jerk,      0.0f, 0.0f, 1.0f);
  TRAJ_MinJerk_Vlim_Plan  (&my_min_jerk_vlim, 0.0f, 0.0f, 0.5f);
  TRAJ_Trapezoidal_Plan   (&my_trapezoid,     0.0f, 0.0f, 1.0f, 0.3f);
  TRAJ_SCurveLimits_Plan  (&my_scurve,        0.0f, 0.0f, 0.5f, 2.0f, 10.0f);

  trajectory.Complete = 1;   /* start in idle — no motion until commanded */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(DEBUG_MODE == 0){
      Modbus_Protocal_Worker();
      Robot_State_Process();
    }
//	  PWM(f32_buffer[0]);
    int_buffer[9] = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
    if(ref_pos.flag_ready == 0){
    SYSTEM_STATE_Homing(&ref_pos);  
    }
    Robot_State_Process();
    // SYSTEM_STATE_Joystick_Update(&robot_joy);
    // SYSTEM_STATE_PilotRamp_SetExclusive(&robot_ramp, int_buffer[1]);
    // if(int_buffer[5] == 1){
    // 	TRAJ_Start();
    // 	int_buffer[5] = 0;
    // }
    // if(int_buffer[6] == 1){
    //     	sys_state.cur_pos += convert_degree2rad(int_buffer[7]);
    //     	int_buffer[6] = 0;
    //     }
    // if(int_buffer[10] == 1){
    // 	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
    // 	int_buffer[10] = 0;
    // }
    // if(int_buffer[11] == 1){
    //     	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
    //     	int_buffer[11] = 0;
    //     }
    f32_buffer[0] = Kalman.X[0]-encoder.encoder_rad;

    //    PWM(f32_buffer[8]);
//	  BaseSystem_SendMotionStatus(Cur_Pos, Cur_Vel, Cur_Acc);
//	  BaseSystem_SendRobotTask(Task,Emergency);
//	  BaseSystem_SendSensorStatus(Grip_up,!Grip_up,Grip_close);
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

/* =========================================================================
 * TRAJ_Start  — plan and arm a trajectory from current position
 *
 * Call this from Robot_State_Process (main loop, not ISR).
 * Sets traj_running = 1 and resets traj_elapsed.
 * Profile is chosen by traj_profile global.
 *
 *   f32_buffer[6]  = target position (rad)        — always used
 *   f32_buffer[7]  = param1 (v_max or t_total)    — profile dependent
 *   f32_buffer[8]  = param2 (accel_time or a_max) — Trapezoid / S-Curve
 *   f32_buffer[9]  = param3 (j_max)               — S-Curve only
 * ========================================================================= */
void TRAJ_Start()
{
    float32_t start  = Kalman.X[0];          /* current estimated position   */
    float32_t target = convert_degree2rad(f32_buffer[6]);        /* destination (rad)            */
    float32_t p1     = f32_buffer[7];        /* v_max or t_total             */
    float32_t p2     = f32_buffer[8];        /* a_max or t_accel             */
    float32_t p3     = f32_buffer[9];        /* j_max                        */

    /* Safety: clamp velocity / time params to non-zero */
    if (p1 < 0.001f) p1 = 0.5f;
    if (p2 < 0.001f) p2 = 2.0f;
    if (p3 < 0.001f) p3 = 10.0f;

    switch (traj_profile)
    {
        default:
        case 0: /* MinJerk — time based, p1 = total time (s) */
            TRAJ_MinJerk_Plan(&my_min_jerk, start, target, p1);
            break;

        case 1: /* MinJerk_Vlim — p1 = v_max (rad/s)  ← best for backlash */
            TRAJ_MinJerk_Vlim_Plan(&my_min_jerk_vlim, start, target, p1);
            break;

        case 2: /* Trapezoidal — p1 = total time (s), p2 = accel time (s) */
            TRAJ_Trapezoidal_Plan(&my_trapezoid, start, target, p1, p2);
            break;

        case 3: /* S-Curve — p1 = v_max, p2 = a_max, p3 = j_max */
            TRAJ_SCurveLimits_Plan(&my_scurve, start, target, p1, p2, p3);
            break;
    }

    traj_elapsed          = 0.0f;
    trajectory.Complete   = 0;
    traj_running          = 1;
    sys_state.cur_pos     = target;   /* store final hold position */
}

/* =========================================================================
 * TRAJ_Step  — evaluate active trajectory at current elapsed time
 *              Returns the Traj_State_t for this tick.
 *              Called from ISR (Robot_Period_Control_Loop).
 * ========================================================================= */
Traj_State_t TRAJ_Step()
{
    traj_elapsed += 0.0005f;   /* 2 kHz inner loop period */

    switch (traj_profile)
    {
        default:
        case 0: return TRAJ_MinJerk_Compute      (&my_min_jerk,      traj_elapsed);
        case 1: return TRAJ_MinJerk_Vlim_Compute (&my_min_jerk_vlim, traj_elapsed);
        case 2: return TRAJ_Trapezoidal_Compute  (&my_trapezoid,     traj_elapsed);
        case 3: return TRAJ_SCurveLimits_Compute (&my_scurve,        traj_elapsed);
    }
}

/* =========================================================================
 * Robot_State_Process  — runs in main loop (non-real-time)
 * ========================================================================= */
void Robot_State_Process(void)
{
    SYSTEM_STATE_Joystick_Update(&robot_joy);

    /* Mode switch PB0: 0 = Joystick, 1 = BaseSystem */
    int state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    sys_state.trust = (state == 0) ? Trust_Joystick : Trust_Basesystem;

    switch (sys_state.cur_state)
    {
        /* ------------------------------------------------------------------ */
        case STATE_WAITING_COMMAND:
            SYSTEM_STATE_PilotRamp_SetExclusive(&robot_ramp, 1);

            // Allow the float buffer trigger to also wake up the state machine
            if (robot_joy.btn_black == 1 || f32_buffer[5] == 1.0f)
            {
                sys_state.cur_state = STATE_RUNNING;
            }
            break;

        /* ------------------------------------------------------------------ */
        case STATE_RUNNING:
            SYSTEM_STATE_PilotRamp_SetExclusive(&robot_ramp, 2);

            /* --- Joystick mode --- */
            if (sys_state.trust == Trust_Joystick)
            {
                if (robot_joy.btn_white_left && trajectory.Complete == 1)
                {
                    f32_buffer[6] = -10.0f;
                    f32_buffer[7] =  0.5f;
                    TRAJ_Start();
                }
                else if (robot_joy.btn_white_right && trajectory.Complete == 1)
                {
                    f32_buffer[6] = 10.0f;
                    f32_buffer[7] = 0.5f;
                    TRAJ_Start();
                }
            }
            /* --- BaseSystem / Autonomous Mode (MATLAB / Modbus) --- */
            else if (sys_state.trust == Trust_Basesystem)
            {
                /* * To launch a profile, set your external buffer values to:
                 * f32_buffer[4] = profile type (0 = MinJerk, 1 = MinJerk_Vlim, etc.)
                 * f32_buffer[6] = target position IN DEGREES
                 * f32_buffer[7] = parameter 1 (v_max or t_total)
                 * f32_buffer[8] = parameter 2 (a_max or t_accel)
                 * f32_buffer[9] = parameter 3 (j_max)
                 * * Then, write 1.0f to f32_buffer[5] to trigger execution!
                 */
                if (f32_buffer[5] == 1.0f && trajectory.Complete == 1)
                {
                    traj_profile = (int)f32_buffer[4]; // Extract profile dynamically
                    f32_buffer[5] = 0.0f;              // Reset trigger immediately to prevent double-firing
                    TRAJ_Start();
                }
                else if (sys_state.trajectory_start_botton == 1 && trajectory.Complete == 1)
                {
                    sys_state.trajectory_start_botton = 0;
                    TRAJ_Start();
                }
            }

            /* --- Abort / Stop Control --- */
            // If you write -1.0f to f32_buffer[5], it will issue an emergency stop
            if (robot_joy.btn_red == 1 || robot_joy.soft_stop == 1 || f32_buffer[5] == -1.0f)
            {
                trajectory.Complete = 1;
                traj_running        = 0;
                f32_buffer[5]       = 0.0f; // Clear trigger
                sys_state.cur_state = STATE_WAITING_COMMAND;
            }
            break;

        /* ------------------------------------------------------------------ */
        default:
            sys_state.cur_state = STATE_WAITING_COMMAND;
            break;
    }
}

/* =========================================================================
 * HAL_GPIO_EXTI_Callback
 * ========================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_10)
    {
        uint16_t captured_counts = (uint16_t)__HAL_TIM_GET_COUNTER(encoder.htim_encoder);
        if (ref_pos.proximity_flag == 0)
        {
            ref_pos.instant_detect = captured_counts;
            ref_pos.proximity_flag = 1;
        }
    }
}

/* =========================================================================
 * Robot_Period_Control_Loop  — 2 kHz ISR
 * ========================================================================= */
void Robot_Period_Control_Loop(void)
{
    static int ending_start_flag = 0;

    /* 1. Encoder */
    SYSTEM_STATE_Encoder_Compute(&encoder);

    /* 2. Homing phase — bypass control, just drive homing voltage */
    if (ref_pos.flag_ready == 0)
    {
        vout = ref_pos.vout;
        PWM(vout,0);
        KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vout);
        ending_start_flag = 1;
        return;
    }

    /* 3. First tick after homing — reset Kalman state to encoder position */
    if (ending_start_flag == 1)
    {
        ending_start_flag   = 0;
        Kalman.X[0]         = encoder.encoder_rad;
        Kalman.X[1]         = 0.0f;
        Kalman.X[2]         = 0.0f;
        Kalman.X[3]         = 0.0f;
        vout = 0;
    }

    /* 4. Kalman predict + update */
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vout);

    /* 5. Update Cascade states from Kalman */
    Controller.pos_state         = Kalman.X[0];
    Controller.velocity_state    = Kalman.X[1];
    Controller.disturbance_state = Kalman.X[2];

    /* 6. Trajectory + Cascade compute */
    if (traj_running)
    {
        /* Evaluate the active trajectory profile this tick */
        Traj_State_t ref = TRAJ_Step();

        /* Feed position + velocity feedforward into cascade */
        CASCADE_Compute(&Controller, ref.pos, ref.velo);

        /* Detect completion */
        if (ref.Complete == 1)
        {
            trajectory.Complete = 1;
            traj_running        = 0;
            /* Hold last position — keep calling CASCADE_Compute with final pos */
        }
    }
    else
    {
        /* Trajectory idle — hold last commanded position, zero velocity FF */
        CASCADE_Compute(&Controller, sys_state.cur_pos, 0.0f);
    }
//    CASCADE_Controller_Compute_Velocity(&Inner, Velo_debug, Kalman.X[1]);
//    CASCADE_Disturbance_Compute(&Controller);
//    CASCADE_Ref_Compute(&Controller);
    vout = Controller.voltage_output;
//    vout = Inner.out+ff.disturbance_value;
    PWM(vout,0.45f);


    /* 7. UART telemetry — 6 doubles: pos_est, vel_est, dist_est, cur_est, encoder, traj_pos */
    double Data_double[6];
    Data_double[0] = (double)Kalman.X[0];
    Data_double[1] = (double)Kalman.X[1];
    Data_double[2] = (double)Kalman.X[2];
    Data_double[3] = (double)Kalman.X[3];
    Data_double[4] = (double)ff.disturbance_value;
    Data_double[5] = (double)vout;

    if (hlpuart1.gState == HAL_UART_STATE_READY)
    {
        Txbuffer[0] = 0xFF;
        memcpy(&Txbuffer[1], Data_double, 48);
        Txbuffer[49] = 0x0F;
        HAL_UART_Transmit_DMA(&hlpuart1, Txbuffer, 50);
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
  while (1)
  {
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
