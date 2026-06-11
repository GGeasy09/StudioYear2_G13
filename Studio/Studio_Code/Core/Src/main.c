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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BaseSystem.h"
#include "kalman.h"
#include "trajectory.h"
#include "cascade.h"
#include "system_state.h"
#include "elec_cabient.h"
#include "robot_process.h"
#include "gripper_can.h"   /* CAN gripper driver — active when GRIPPER_USE_CAN=1 */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_MODE 0   /* 0 = normal, 1 = trajectory-tuning: P2P auto-fires gripper on arrival */

//Inner Loop ---------  (synced from LAB3 lab_config.h)
#define Kp_inner 10.0f
#define Ki_inner 25.0f
#define Kd_inner 0.0f
//Outer Loop ---------  (synced from LAB3 lab_config.h)
#define Kp_outer 3.5f
#define Ki_outer 2.0f
#define Kd_outer 0.0f

/* ---- Robot geometry / sequencing constants ---- */
#define HOLE_COUNT          72      /* 360 / 5 = 72 holes around the disc   */
#define GRIPPER_TIMEOUT_MS  5000U   /* max wait for a gripper pick/place    */

/* ---- Default trajectory timing (seconds) ---- */

/* Trajectory profile IDs, P2P_TUNE_*, and TRAJ_* defines live in robot_process.h */

#define static_friction_ff   0.70f   /* synced from LAB3 CFG_FRICTION_STATIC  */
#define dynamic_friction_ff  0.50f   /* synced from LAB3 CFG_FRICTION_DYNAMIC */
#define friction_velo_thresh 0.08f   /* rad/s — synced from LAB3 FRICTION_VELO_THRESH */
#define friction_pos_thresh  0.0087f /* rad (~0.5 deg) */

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
FrictionFF         friction;
Encoder            encoder;
Proximity            ref_pos;
Joystick             robot_joy;
Pilot_ramp           robot_ramp;
/* ---- Trajectory manager (owns all planners, state, profile) ---- */
TrajManager traj_mgr;

/* ---- Emergency ---- */
uint8_t   emergency_latch = 0;   /* 1 = emergency was triggered, waiting for release */

/* ---- Pick-place sequence state (SeqState now in robot_process.h) ---- */
SeqState seq = {0};

/* ---- Test mode state (TestState now in robot_process.h) ---- */
TestState test = {0};

/* ---- Joystick button edge-detection (JoyEdges now in robot_process.h) ---- */
JoyEdges joy_prev = {0};

/* ---- Joystick gripper action flags (mode 1) ---- */
uint8_t joy_picking  = 0;   /* 1 = Gripper_Pick running  */
uint8_t joy_placing  = 0;   /* 1 = Gripper_Place running */
uint8_t p2p_grip_pending = 0; /* DEBUG_MODE 1: P2P move armed to fire gripper on arrival */

/* ---- Color button 2s hold → go to robot reference ---- */
uint32_t color_hold_start = 0;

/* ---- Gripper ---- */
Gripper   robot_gripper = {0};

/* ---- Motor ---- */
MOTOR_PARAMS my_motor = {
   .kt  = 2.2461f,
   .km  = 2.2461f,
   .J   = 8.6701E-1f,
   .B   = 0.4308f,
   .R   = 0.4821323156f,
   .L   = 0.0002893301219f,
   .tau      = 0.05f,    /* legacy (unused now that ref/dist are split) */
   .tau_ref  = 0.01f,    /* reference FF: small/fast -> low lag */
   .tau_dist = 0.01f     /* disturbance FF: synced from LAB3 CFG_TAU_DIST */
};
float32_t f32_buffer[10];
float32_t vout;
float32_t hold_ki_outer = Ki_outer;     /* outer ki — writable via debugger to match lab.Ki_out  */
float32_t vin_kalman  = 0.0f;   /* PID + ref_FF only — fed to Kalman, no dist_FF/friction */
int moving;
float32_t actual_vin = 0.0f;   /* pre-friction vin for reference logging */
float32_t friction_feedforward = 0.0f;
int prox;
int reed1;
int reed2;
int reed3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Robot_State_Process(void);
void Robot_Period_Control_Loop();
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
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM20_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // 1. Start Hardware Timers
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    PWM(0.0f, 0.0f);   /* ensure motor is off before any init completes */
    HAL_TIM_RegisterCallback(&htim20, HAL_TIM_PERIOD_ELAPSED_CB_ID, Robot_Period_Control_Loop);
    /* TIM20 (control loop ISR) started AFTER homing completes */
    // 2. Initialize Encoder Pointers
    SYSTEM_STATE_Encoder_Init(&htim2, &htim3, &encoder);
    TrajManager_Init(&traj_mgr, &htim2);
    // 3. Setup initial state machine defaults
    sys_state.cur_state = STATE_WAITING_COMMAND; /* set to HOMING below after TRAJ_Start */
    sys_state.trust     = Trust_Basesystem;
    sys_state.home_pos  = 0.0f;
  hmodbus.huart = &huart2; // เปลี่ยนเป็น huart ของคุณ (เช่น huart1 หรือ huart2)
  hmodbus.htim = &htim16;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize = BASE_SYSTEM_REG_COUNT;

  Modbus_init(&hmodbus, registerFrame);

  /* ---- CAN Gripper Init (active only when GRIPPER_USE_CAN == 1) ---- */
#if (GRIPPER_USE_CAN == 1)
  Gripper_CAN_Init();   /* configures filters, enables RX IRQ, starts FDCAN, sends first heartbeat */
#endif

  __HAL_TIM_SET_COUNTER(&htim3, 30000);

  /* ====================================================================
   * 4. ACTIVATE ALGORITHM INITIALIZATIONS HERE
   * ==================================================================== */
  KALMAN_Multi_Model_Init(&Kalman, 5.33e-11f, 5.00e-12f, 3e-8f, &my_motor);
  CASCADE_Controller_Init(Kp_inner, Ki_inner, Kd_inner, &Inner, -10.5f, 10.5f, 0.0005f);
  CASCADE_Controller_Init(Kp_outer, Ki_outer, Kd_outer, &Outer, -4.0f, 4.0f, 0.002f);
  CASCADE_Cascade_Start(&Controller, &Inner, &Outer, &ff, &my_motor, &Kalman);
  CASCADE_Friction_Init(&friction, static_friction_ff, dynamic_friction_ff,
                        friction_velo_thresh, friction_pos_thresh);

  /* TrajManager_Init already pre-armed all planners and set Complete=1 */

  /* Homing uses proximity sensor state machine.
   * ISR (TIM20) stays off until homing is complete. */
  ref_pos.flag_ready      = 0;
  ref_pos.state_detection = 0;
  ref_pos.proximity_flag  = 0;
  sys_state.cur_state     = STATE_HOMING;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Gripper_ReadState(&robot_gripper);       /* GPIO path: reads pins. CAN path: no-op (updated by IRQ). */
#if (GRIPPER_USE_CAN == 1)
	  Gripper_CAN_Tick(&robot_gripper);        /* heartbeat every 500 ms + opto poll every 200 ms */
#endif
	  if(DEBUG_MODE == 0 || DEBUG_MODE == 1){
	      Modbus_Protocal_Worker();
	      Robot_State_Process();

	      /* Update position derived values relative to home */
	      sys_state.cur_pos_degree = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos);
	      sys_state.cur_hole_index = (int16_t)roundf(sys_state.cur_pos_degree / DEG_PER_HOLE) % HOLE_COUNT;
	      if (sys_state.cur_hole_index < 0)
	          sys_state.cur_hole_index += HOLE_COUNT;

	      /* Motion feedback: position [deg] = ACTUAL encoder position (relative to BS home),
	       * velocity [rad/s], acceleration [rad/s²] */
	      BaseSystem_SendMotionStatus(
	          SYSTEM_STATE_convert_rad2degree(encoder.encoder_rad - sys_state.basesystem_home_pos),
	          Kalman.X[1],
	          Kalman.accel_estimate);

	      /* Task bits (reg 0x27) — match README bit map:
	       *   0x0001=Homing  0x0008=Go Point  0x0000=Idle */
	      uint16_t cur_task = 0;
	      if      (sys_state.cur_state == STATE_HOMING) cur_task = 0x0001;
	      else if (traj_mgr.running)                         cur_task = 0x0008;
	      else                                           cur_task = 0x0000;

	      /* Emergency = soft-stop active */
	      bool is_emergency = (robot_joy.soft_stop == 1) ||
	                          (BaseCmd.Soft_Stop_Req && sys_state.trust == Trust_Basesystem);
	      BaseSystem_SendRobotTask(cur_task, is_emergency);

	      /* Gripper status reporting mode:
	       *   0 = command mode  — read from output command variables in struct (default)
	       *   1 = reed switch   — read from physical sensors (is_full_open, is_full_close, is_up)
	       *                       NOTE: reed switch mode is pre-code, not yet verified */
	      #define GRIPPER_STATUS_MODE  0   /* 0 = command mode (default), 1 = reed switch */

	      bool grip_up, grip_down, grip_closed;

	      #if (GRIPPER_STATUS_MODE == 0)
	          /* Mode 0: derive status from output command state */
	          grip_up     = robot_gripper.grip_up;
	          grip_down   = !robot_gripper.grip_up;
	          grip_closed = robot_gripper.grip_open;

	      #else
	          /* Mode 1: read physical reed switch sensors (pre-code — verify wiring/polarity) */
	          Gripper_ReadState(&robot_gripper);   /* update sensor fields from GPIO */
	          grip_up     = robot_gripper.is_up;
	          grip_down   = !robot_gripper.is_up;
	          grip_closed = robot_gripper.is_full_close;
	          /* Note: is_full_open available as robot_gripper.is_full_open if needed */
	      #endif

	      BaseSystem_SendSensorStatus(grip_up, grip_down, grip_closed);
	    }

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
 * HAL_FDCAN_RxFifo0MsgPendingCallback — CAN RX interrupt
 * Forwards incoming frames to the gripper CAN driver.
 * Active only when GRIPPER_USE_CAN == 1; zero overhead otherwise.
 * ========================================================================= */
#if (GRIPPER_USE_CAN == 1)
void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        Gripper_CAN_ProcessRx(&robot_gripper, &rxHeader, rxData);
}
#endif

/* =========================================================================
 * TRAJ_Start  — plan and arm a trajectory from current position
 *
 * Call this from Robot_State_Process (main loop, not ISR).
 * Sets traj_mgr.running = 1 and resets traj_elapsed.
 * Profile is chosen by traj_profile global.
 *
 *   f32_buffer[6]  = target position (rad)        — always used
 *   f32_buffer[7]  = param1 (v_max or t_total)    — profile dependent
 *   f32_buffer[8]  = param2 (accel_time or a_max) — Trapezoid / S-Curve
 *   f32_buffer[9]  = param3 (j_max)               — S-Curve only
 * ========================================================================= */
/* Robot_State_Process, its helpers, and TRAJ_Plan now live in robot_process.c */

/* =========================================================================
 * HAL_GPIO_EXTI_Callback
 * ========================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)   /* home/prox sensor moved to PA0 (EXTI0) */
    {
        uint16_t captured_counts = (uint16_t)__HAL_TIM_GET_COUNTER(encoder.htim_encoder);
        if (ref_pos.proximity_flag == 0)
        {
                ref_pos.instant_detect = captured_counts;
            ref_pos.proximity_flag = 1;
        }
    }
}

/* USER CODE BEGIN 4 */

/* =========================================================================
 * Hold position constants
 * ========================================================================= */
#define HOLD_POS_DEADZONE_DEG   0.12f     /* outer PID killed, integrals bleed */
#define HOLD_INTEGRAL_DECAY     0.95f     /* integral bleed factor per tick    */
#define HOLD_DISTURBANCE_SCALE  1.0f      /* dist_FF runtime multiplier — tune live via debugger */

/* =========================================================================
 * Robot_HoldPosition — cascade hold at sys_state.cur_pos
 *
 * Called every tick while traj_mgr.running == 0.
 * Writes vin_kalman, vout, friction_feedforward.
 *
 * Dead-zone logic:
 *   |err| > 0.12°  outer position PID active
 *   |err| ≤ 0.12°  outer PID = 0, integrals bleed ×0.95, dist_FF decays ×0.9
 * ========================================================================= */
static void Robot_HoldPosition(void)
{
    Controller.traj_running = 0;

    float32_t err_deg = fabsf((sys_state.cur_pos - Kalman.X[0]) * 57.295f);

    /* ---- Outer position PID — decimated to match CASCADE_Compute (every 4 ticks) ---- */
    Controller.loop_counter++;
    if (Controller.loop_counter >= 4)   /* 4 = OUTER_DECIMATE in cascade.c */
    {
        Controller.loop_counter = 0;
        if (err_deg > HOLD_POS_DEADZONE_DEG)
        {
            Outer.ki = hold_ki_outer;
            CASCADE_Controller_Compute_Position(&Outer, sys_state.cur_pos, encoder.encoder_rad);
        }
        else
        {
            Outer.out = 0.0f;
        }
    }
    /* Integral bleed runs every tick regardless of decimation */
    if (err_deg <= HOLD_POS_DEADZONE_DEG)
    {
        Outer.integral *= HOLD_INTEGRAL_DECAY;
        Inner.integral *= HOLD_INTEGRAL_DECAY;
    }

    /* ---- Inner velocity PID (every tick) ---- */
    CASCADE_Controller_Compute_Velocity(&Inner, Outer.out, Kalman.X[1]);

    /* ---- Reference FF — fed with velocity command (Outer.out ≈ 0 at rest) ---- */
    Controller.velo_setpoint = Outer.out;
    CASCADE_Ref_Compute(&Controller);

    /* ---- Disturbance FF — disturbance_state already set by CASCADE_Update ---- */
    CASCADE_Disturbance_Compute(&Controller);

    /* ---- Disturbance scale: decay Kalman X[2] and FF state when settled ---- */
    if (err_deg < 0.1f)
    {
        Kalman.X[2] *= 0.9f;   /* same as CASCADE_Compute internal scale */
        Controller.feedforward->disturbance_value *= 0.9f;
        Controller.feedforward->dist_y_prev       *= 0.9f;
        Controller.feedforward->dist_u_prev       *= 0.9f;
    }

    /* ---- Disturbance scale (tune HOLD_DISTURBANCE_SCALE or hold_ki_outer live) ---- */
    Controller.feedforward->disturbance_value *= HOLD_DISTURBANCE_SCALE;

    /* ---- Clamp dist_FF to ±5 V (same limit as CASCADE_Compute) ---- */
    if      (Controller.feedforward->disturbance_value >  5.0f) Controller.feedforward->disturbance_value =  5.0f;
    else if (Controller.feedforward->disturbance_value < -5.0f) Controller.feedforward->disturbance_value = -5.0f;

    vin_kalman = Inner.out;
    vout       = vin_kalman + Controller.feedforward->disturbance_value;
    friction_feedforward = CASCADE_Friction_Compute(
        &friction, Kalman.X[1], sys_state.cur_pos - encoder.encoder_rad);


    if      (vout >  0.8f) {vout =  0.8f;vin_kalman = 0.8f;}
        else if (vout < -0.8f) {vout =  -0.8f;vin_kalman = -0.8f;}
}

/* =========================================================================
 * Robot_Period_Control_Loop — TIM20 ISR, 2 kHz
 * ========================================================================= */
void Robot_Period_Control_Loop(TIM_HandleTypeDef *htim)
{
    (void)htim;

    /* 1. Read encoder */
    SYSTEM_STATE_Encoder_Compute(&encoder);

    /* 2. Kalman — feed PID + ref_FF only (dist_FF and friction bypass observer) */
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vin_kalman);

    /* 3. Sync Kalman states into cascade struct — pos_state = raw encoder (matches LAB3) */
    CASCADE_Update(&Controller,
                   encoder.encoder_rad,
                   Kalman.X[1],
                   Kalman.X[3],
                   Kalman.X[2],
                   encoder.encoder_rad);

    if (traj_mgr.running)
    {
        /* ---- TRAJECTORY PHASE ---- */
        Traj_State_t ref = TrajManager_Step(&traj_mgr);

        Controller.traj_running = 1;
        CASCADE_Compute(&Controller, ref.pos, ref.velo);

        friction_feedforward = CASCADE_Friction_Compute(
            &friction, Kalman.X[1], ref.pos - Kalman.X[0]);

        vin_kalman = Inner.out + Controller.feedforward->reference_value;
        vout       = vin_kalman + Controller.feedforward->disturbance_value;
    }
    else
    {
        /* ---- HOLD POSITION PHASE ---- */
        Robot_HoldPosition();
    }

    moving     = friction.moving;
    actual_vin = vin_kalman;

    PWM(vout, friction_feedforward);
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
