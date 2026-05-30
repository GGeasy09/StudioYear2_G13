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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_MODE 0

//Inner Loop ---------
#define Kp_inner 0.52259f
#define Ki_inner 10.2259f
#define Kd_inner 0.0f
//Outer Loop ---------
#define Kp_outer 2.0f
#define Ki_outer 0.0f
#define Kd_outer 1.0f

/* ---- Robot geometry / sequencing constants ---- */
#define DEG_PER_HOLE        5.0f    /* one hole = 5 degrees                 */
#define HOLE_COUNT          72      /* 360 / 5 = 72 holes around the disc   */
#define GRIPPER_TIMEOUT_MS  5000U   /* max wait for a gripper pick/place    */

/* ---- Default trajectory timing (seconds) ---- */

/* ---- Trajectory profile IDs (traj_profile) ---- */
#define PROFILE_MINJERK     0
#define PROFILE_MINJERK_VLIM 1
#define PROFILE_TRAPEZOID   2
#define PROFILE_SCURVE      3

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
/* ---- Trajectory manager (owns all planners, state, profile) ---- */
TrajManager traj_mgr;

/* ---- Emergency ---- */
uint8_t   emergency_latch = 0;   /* 1 = emergency was triggered, waiting for release */

/* ---- Pick-place sequence state ---- */
typedef struct {
    int active;   /* 1 = sequence currently running       */
    int step;     /* current pair index (0..Pairs-1)      */
    int phase;    /* 0=go pick 1=gripper pick 2=go place 3=gripper place */
} SeqState;
SeqState seq = {0};

/* ---- Test mode state ---- */
typedef struct {
    int      active;        /* 1 = test running                                      */
    int      repeat;        /* completed round-trips so far                          */
    int      phase;         /* PERFORM: 0=go 1=delay 2=back 3=done                  */
                            /* PRECISION: 0=go 1=delay_after_go 2=back 3=delay_after_back */
    uint32_t delay_start;   /* timestamp for inter-move delay                        */
} TestState;
TestState test = {0};

/* ---- Joystick button edge-detection (previous raw states) ---- */
typedef struct {
    uint8_t white_left;
    uint8_t white_right;
    uint8_t black;
    uint8_t blue;
    uint8_t yellow;
    uint8_t red;
} JoyEdges;
JoyEdges joy_prev = {0};

/* ---- Joystick gripper action flags (mode 1) ---- */
uint8_t joy_picking  = 0;   /* 1 = Gripper_Pick running  */
uint8_t joy_placing  = 0;   /* 1 = Gripper_Place running */

/* ---- Color button 2s hold → go to robot reference ---- */
uint32_t color_hold_start = 0;

/* ---- Gripper ---- */
Gripper   robot_gripper = {0};

/* ---- Motor ---- */
MOTOR_PARAMS my_motor = {
   .kt  = 0.20906532f,
   .km  = 0.2316f,
   .J   = 0.107E0f,
   .B   = 0.995f,
   .R   = 0.4821323156f,
   .L   = 0.0002893301219f,
   .tau = 0.001f          /* 10 ms — safe starting tau (was 1 ms → too aggressive) */
};

float32_t f32_buffer[10];
float32_t vout;

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

  __HAL_TIM_SET_COUNTER(&htim3, 30000);

  /* ====================================================================
   * 4. ACTIVATE ALGORITHM INITIALIZATIONS HERE
   * ==================================================================== */
  KALMAN_Multi_Model_Init(&Kalman, 3e-8f,9.9e-5f, 8e-4f, &my_motor);
  CASCADE_Controller_Init(Kp_inner, Ki_inner, Kd_inner, &Inner, -12.0f, 12.0f, 0.0005f);
  CASCADE_Controller_Init(Kp_outer, Ki_outer, Kd_outer, &Outer, -4.0f, 4.0f, 0.005f);
  CASCADE_Cascade_Start(&Controller, &Inner, &Outer, &ff, &my_motor, &Kalman);

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

	  if(DEBUG_MODE == 0){
	      Modbus_Protocal_Worker();
	      Robot_State_Process();

	      /* Update position derived values relative to home */
	      sys_state.cur_pos_degree = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos);
	      sys_state.cur_hole_index = (int16_t)roundf(sys_state.cur_pos_degree / DEG_PER_HOLE) % HOLE_COUNT;
	      if (sys_state.cur_hole_index < 0)
	          sys_state.cur_hole_index += HOLE_COUNT;

	      /* Motion feedback: position [deg], velocity [rad/s], acceleration [rad/s²] */
	      BaseSystem_SendMotionStatus(
	          SYSTEM_STATE_convert_rad2degree(Kalman.X[0] - sys_state.basesystem_home_pos),
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

	      /* Gripper status from struct variables */
	      bool grip_up     = robot_gripper.grip_up;    /* 1=up,    0=down  */
	      bool grip_down   = !robot_gripper.grip_up;   /* 1=down,  0=up    */
	      bool grip_closed = robot_gripper.grip_open;  /* 1=close, 0=open  */
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
/* =========================================================================
 * TRAJ_Plan — local wrapper: arms a move and updates sys_state.cur_pos
 * Calls TrajManager_Plan then latches the target into sys_state.cur_pos
 * ========================================================================= */
static void TRAJ_Plan(int profile, float32_t target_deg,
                      float32_t p1, float32_t p2, float32_t p3)
{
    TrajManager_Plan(&traj_mgr, profile, target_deg, p1, p2, p3, Kalman.X[0]);
    sys_state.cur_pos = traj_mgr.cur_pos_rad;
}

/* =========================================================================
 * Robot_State_Process  — runs in main loop (non-real-time)
 * ========================================================================= */
void Robot_State_Process(void)
{
    SYSTEM_STATE_Joystick_Update(&robot_joy);

    /* =========================================================================
     * EMERGENCY BUTTON — PA4 pull-up: 0 = pressed, 1 = released
     * Overrides all states — checked before anything else
     * ========================================================================= */
    uint8_t emer_pin = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    if (emer_pin == 0 && !emergency_latch)
    {
        /* Button just pressed — emergency triggered */
        emergency_latch = 1;
        HAL_TIM_Base_Stop_IT(&htim20);   /* stop ISR so it can't override PWM */
        traj_mgr.running = 0;
        seq.active   = 0;
        PWM(0.0f, 0.0f);                 /* cut motor immediately */
        PilotRamp_SetPower(&robot_ramp, 0);  /* Emergency LED ON, Power OFF */
    }
    else if (emer_pin == 1 && emergency_latch)
    {
        /* Button released — recover: seed position, restart homing */
        emergency_latch = 0;

        SYSTEM_STATE_Encoder_Compute(&encoder);
        Kalman.X[0]  = encoder.encoder_rad;   /* position from encoder */
        Kalman.X[1]  = 0.0f;                  /* clear velocity */
        Kalman.X[2]  = 0.0f;                  /* clear disturbance */
        Kalman.X[3]  = 0.0f;
        sys_state.cur_pos = encoder.encoder_rad;

        /* Restart homing sequence */
        ref_pos.flag_ready      = 0;
        ref_pos.state_detection = 0;
        ref_pos.proximity_flag  = 0;
        sys_state.cur_state     = STATE_HOMING;
    }

    /* Skip state machine while emergency is active */
    if (emergency_latch) return;

    /* =========================================================================
     * SOFT STOP — top priority after emergency, ignores trust mode
     * ========================================================================= */
    bool soft_stop = (robot_joy.soft_stop == 1) || (BaseCmd.Soft_Stop_Req == 1);
    if (soft_stop)
    {
        traj_mgr.state.Complete = 1;
        traj_mgr.running        = 0;
        seq.active          = 0;
        test.active         = 0;
        PilotLamp_SetWhite(0);
        PilotRamp_SetPower(&robot_ramp, 0);   /* Power OFF */
        sys_state.cur_state = STATE_WAITING_COMMAND;
        return;
    }

    /* Mode switch PB0: 0 = Joystick, 1 = BaseSystem */
    int sw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    sys_state.trust = (sw == 0) ? Trust_Joystick : Trust_Basesystem;

    /* =========================================================================
     * GRIPPER MANUAL — works in any state / any trust mode
     * ========================================================================= */
    if (BaseCmd.flag_gripper_manual == 1)
    {
        BaseCmd.flag_gripper_manual = 0;
        switch (BaseCmd.Gripper_Manual)
        {
            case CMD_GRIPPER_UP:    Gripper_SetUp  (&robot_gripper, 1); break;
            case CMD_GRIPPER_DOWN:  Gripper_SetUp  (&robot_gripper, 0); break;
            case CMD_GRIPPER_OPEN:  Gripper_SetOpen(&robot_gripper, 0); break;
            case CMD_GRIPPER_CLOSE: Gripper_SetOpen(&robot_gripper, 1); break;
        }
    }

    /* =========================================================================
     * GRIPPER SEQ (pick / place) — 1 = pick, 2 = place
     * ========================================================================= */
    if (BaseCmd.flag_gripper_seq == 1)
    {
        BaseCmd.flag_gripper_seq   = 0;
        robot_gripper.action_phase = 0;
        if (BaseCmd.Gripper_Seq == 1) { joy_picking = 1; joy_placing = 0; }
        if (BaseCmd.Gripper_Seq == 2) { joy_placing = 1; joy_picking = 0; }
    }

    /* Run pick/place state machine (shared with joystick mode 1) */
    if (joy_picking)
    {
        if (Gripper_Pick(&robot_gripper))  joy_picking = 0;
    }
    else if (joy_placing)
    {
        if (Gripper_Place(&robot_gripper)) joy_placing = 0;
    }

    /* SET_HOME from BaseSystem — snapshot current pos as BS coordinate origin */
    if (BaseCmd.flag_sethome_execute == 1)
    {
        BaseCmd.flag_sethome_execute  = 0;
        sys_state.basesystem_home_pos = Kalman.X[0];
    }

    /* =========================================================================
     * COLOR BUTTON 2s HOLD — move to robot reference (0°) regardless of mode
     * Any of black/blue/red/yellow held for 2s triggers the move
     * ========================================================================= */
    uint8_t any_color_held = (robot_joy.raw_btn_black  == 0)
                           || (robot_joy.raw_btn_blue   == 0)
                           || (robot_joy.raw_btn_red    == 0)
                           || (robot_joy.raw_btn_yellow == 0);

    if (any_color_held)
    {
        if (color_hold_start == 0)
            color_hold_start = HAL_GetTick();

        if (HAL_GetTick() - color_hold_start >= 2000)
        {
            color_hold_start = 0;
            traj_mgr.running     = 0;
            seq.active       = 0;
            test.active      = 0;
            TRAJ_Plan(PROFILE_MINJERK, 0.0f, TrajManager_DynTime(0.0f, Kalman.X[0]), 0.0f, 0.0f);   /* robot reference = 0° */
            sys_state.cur_state = STATE_RUNNING;
        }
    }
    else
    {
        color_hold_start = 0;   /* reset if released */
    }


    switch (sys_state.cur_state)
    {
        /* ------------------------------------------------------------------ */
        case STATE_HOMING:
            PilotRamp_SetPower(&robot_ramp, 1);
            PilotRamp_SetAuto(&robot_ramp, 0);   /* Manual ON during homing */

            /* Proximity sensor homing — owns PWM directly, ISR is off */
            SYSTEM_STATE_Homing(&ref_pos);
            PWM(ref_pos.vout, 0.0f);

            /* flag_ready=1 means sensor found center, encoder counter corrected */
            if (ref_pos.flag_ready == 1)
            {

                /* Sync encoder then seed Kalman from real position */
                SYSTEM_STATE_Encoder_Compute(&encoder);
                Kalman.X[0]         = encoder.encoder_rad;
                Kalman.X[1]         = 0.0f;
                Kalman.X[2]         = 0.0f;
                Kalman.X[3]         = 0.0f;
                sys_state.cur_pos   = 0.0f;
                sys_state.home_pos  = 0.0f;
                sys_state.cur_state = STATE_WAITING_COMMAND;
                PWM(0,0);
                /* Start control loop ISR now that reference is established */
                HAL_TIM_Base_Start_IT(&htim20);
            }
            break;


        /* ------------------------------------------------------------------ */
        case STATE_WAITING_COMMAND:
            PilotRamp_SetPower(&robot_ramp, 1);

            /* ---- Joystick mode ---- */
            if (sys_state.trust == Trust_Joystick)
            {
                /* White buttons — always same regardless of mode */
                if (robot_joy.raw_btn_white_left == 0 && joy_prev.white_left == 1)
                {
                    float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos) - DEG_PER_HOLE;
                    TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                    sys_state.cur_state = STATE_RUNNING;
                }
                else if (robot_joy.raw_btn_white_right == 0 && joy_prev.white_right == 1)
                {
                    float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos) + DEG_PER_HOLE;
                    TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                    sys_state.cur_state = STATE_RUNNING;
                }

                /* ---- Mode 0: direct gripper control ---- */
                if (robot_joy.mode == 0)
                {
                    if (robot_joy.raw_btn_black  == 0 && joy_prev.black  == 1)
                        Gripper_SetOpen(&robot_gripper, 0);   /* Open  */
                    if (robot_joy.raw_btn_blue   == 0 && joy_prev.blue   == 1)
                        Gripper_SetOpen(&robot_gripper, 1);   /* Close */
                    if (robot_joy.raw_btn_yellow == 0 && joy_prev.yellow == 1)
                        Gripper_SetUp(&robot_gripper, 1);     /* Up    */
                    if (robot_joy.raw_btn_red    == 0 && joy_prev.red    == 1)
                        Gripper_SetUp(&robot_gripper, 0);     /* Down  */
                }

                /* ---- Mode 1: home / pick / place ---- */
                if (robot_joy.mode == 1)
                {
                    /* Black — set home to current position */
                    if (robot_joy.raw_btn_black == 0 && joy_prev.black == 1)
                        sys_state.home_pos = sys_state.cur_pos;

                    /* Blue — go to home position */
                    if (robot_joy.raw_btn_blue == 0 && joy_prev.blue == 1)
                    {
                        float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.home_pos);
                        TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                        sys_state.cur_state = STATE_RUNNING;
                    }

                    /* Red — start gripper pick */
                    if (robot_joy.raw_btn_red == 0 && joy_prev.red == 1)
                    {
                        joy_picking          = 1;
                        joy_placing          = 0;
                        robot_gripper.action_phase = 0;
                    }

                    /* Yellow — start gripper place */
                    if (robot_joy.raw_btn_yellow == 0 && joy_prev.yellow == 1)
                    {
                        joy_placing          = 1;
                        joy_picking          = 0;
                        robot_gripper.action_phase = 0;
                    }

                    /* pick/place state machine runs at top level each tick */
                }

                /* edge-detection history updated below, outside state machine */
            }

            /* BaseSystem mode commands */
            if (sys_state.trust == Trust_Basesystem)
            {
                /* SET_HOME is joystick-only — BaseSystem command ignored here */
                if (BaseCmd.Target_Mode == CMD_MODE_HOME)
                {
                    /* Actual planning happens in STATE_RUNNING's HOME handler */
                    sys_state.cur_state = STATE_RUNNING;
                }
                else if (BaseCmd.Target_Mode == CMD_MODE_JOG)
                {
                    PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON */
                    if (BaseCmd.flag_jog_execute == 1)
                    {
                        /* Don't clear flag here — STATE_RUNNING clears it after executing */
                        sys_state.cur_state = STATE_RUNNING;
                    }
                }
                else if (BaseCmd.Target_Mode == CMD_MODE_AUTO)
                {
                    PilotRamp_SetAuto(&robot_ramp, 0);   /* Auto ON */
                    if (BaseCmd.flag_p2p_execute == 1 || BaseCmd.flag_seq_execute == 1)
                        sys_state.cur_state = STATE_RUNNING;
                }
                else if (BaseCmd.Target_Mode == CMD_MODE_TEST)
                {
                    PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON */
                    if (BaseCmd.flag_test_execute == 1)
                        sys_state.cur_state = STATE_RUNNING;
                }
            }
            break;

        /* ------------------------------------------------------------------ */
        case STATE_RUNNING:
            PilotRamp_SetPower(&robot_ramp, 1);
            /* Lamp set per-mode inside handlers — do NOT override here */

            /* ---- BaseSystem commands ---- */
            if (sys_state.trust == Trust_Basesystem)
            {
                /* Lamp reflects active mode: Auto=0, Manual=1 */
                if (BaseCmd.Target_Mode == CMD_MODE_AUTO || BaseCmd.Target_Mode == CMD_MODE_HOME)
                    PilotRamp_SetAuto(&robot_ramp, 0);   /* Auto ON  */
                else
                    PilotRamp_SetAuto(&robot_ramp, 1);   /* Manual ON (JOG / TEST / SET_HOME) */

                /* SET_HOME handled at top level via flag_sethome_execute */
                /* ---- HOME: abort current motion, drive to home position ---- */
                if (BaseCmd.Target_Mode == CMD_MODE_HOME)
                {
                    traj_mgr.running        = 0;
                    seq.active          = 0;
                    test.active         = 0;
                    BaseCmd.Target_Mode = CMD_MODE_IDLE;
                    {
                        float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.home_pos);
                        TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                    }
                    break;
                }
                /* ---- JOG: one step from current position ---- */
                else if (BaseCmd.Target_Mode == CMD_MODE_JOG
                         && traj_mgr.state.Complete == 1
                         && BaseCmd.flag_jog_execute == 1)
                {
                    BaseCmd.flag_jog_execute = 0;
                    registerFrame[0x05].U16 = 0;
                    float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos) + BaseCmd.Jog_Degree;
                    TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                    BaseCmd.Jog_Degree = 0.0f;
                }
                /* ---- P2P: single point-to-point move ---- */
                else if (BaseCmd.Target_Mode == CMD_MODE_AUTO
                         && BaseCmd.flag_p2p_execute == 1
                         && !traj_mgr.running)
                {
                    BaseCmd.flag_p2p_execute = 0;
                    float32_t home_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
                    float32_t target   = (BaseCmd.P2P_Unit == 1)
                                         ? home_deg + BaseCmd.P2P_Target * DEG_PER_HOLE
                                         : home_deg + BaseCmd.P2P_Target;
                    TRAJ_Plan(PROFILE_MINJERK, target, TrajManager_DynTime(target, Kalman.X[0]), 0.0f, 0.0f);
                }
                /* ---- SEQUENCE: pick-place pairs ---- */
                else if (BaseCmd.Target_Mode == CMD_MODE_AUTO
                         && BaseCmd.flag_seq_execute == 1
                         && !seq.active)
                {
                    BaseCmd.flag_seq_execute = 0;
                    seq.active = 1;
                    seq.step   = 0;
                    seq.phase  = 0;
                    {
                        float32_t home_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
                        int16_t   raw      = BaseCmd.PickPlace_Sequence[0];
                        float32_t tgt      = home_deg + (float)abs(raw) * DEG_PER_HOLE;
                        TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                    }
                }
                /* ---- TEST mode trigger ---- */
                else if (BaseCmd.Target_Mode == CMD_MODE_TEST
                         && BaseCmd.flag_test_execute == 1
                         && !test.active
                         && traj_mgr.state.Complete == 1)
                {
                    BaseCmd.flag_test_execute = 0;
                    PilotLamp_SetWhite(1);
                    test.active      = 1;
                    test.phase       = 0;
                    test.repeat      = 0;
                    test.delay_start = 0;

                    /* Decode unit from sign of Test_Repeat_Count:
                     * negative = index mode, positive = degree mode */
                    uint8_t   is_index   = (BaseCmd.Test_Repeat_Count < 0.0f);
                    float32_t test_final = is_index
                                           ? BaseCmd.Test_Final_Pos * DEG_PER_HOLE
                                           : BaseCmd.Test_Final_Pos;

                    if (BaseCmd.Test_Type == CMD_TEST_PERFORM)
                    {
                        /* --- PERFORM: S-Curve, ref → ref+360° → delay 0.5s → ref ---
                         * v_max clamped at 4 rad/s, a_max clamped at 8 rad/s²
                         * j_max = 2.5 × a_max                                        */
                        float32_t vmax    = BaseCmd.Test_Velocity > 0.01f ? BaseCmd.Test_Velocity : 1.0f;
                        float32_t amax    = BaseCmd.Test_Accel    > 0.01f ? BaseCmd.Test_Accel    : 2.0f;
                        if (vmax > 4.0f) vmax = 4.0f;
                        if (amax > 8.0f) amax = 8.0f;
                        float32_t ref_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
                        TRAJ_Plan(PROFILE_SCURVE, ref_deg + 360.0f, vmax, amax, 2.5f * amax);
                    }
                    else
                    {
                        /* --- PRECISION: MinJerk, Init → Final → 1s → back → repeat --- */
                        TRAJ_Plan(PROFILE_MINJERK, test_final, TrajManager_DynTime(test_final, Kalman.X[0]), 0.0f, 0.0f);
                    }
                }
                /* ---- TEST step advance ---- */
                else if (test.active)
                {
                    /* Recalculate converted positions — sign of repeat encodes unit */
                    uint8_t   t_is_index = (BaseCmd.Test_Repeat_Count < 0.0f);
                    float32_t t_init     = t_is_index
                                          ? BaseCmd.Test_Init_Pos  * DEG_PER_HOLE
                                          : BaseCmd.Test_Init_Pos;
                    float32_t t_final    = t_is_index
                                          ? BaseCmd.Test_Final_Pos * DEG_PER_HOLE
                                          : BaseCmd.Test_Final_Pos;
                    int t_repeat_count   = (int)fabsf(BaseCmd.Test_Repeat_Count);
                    float32_t ref_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);

                    if (BaseCmd.Test_Type == CMD_TEST_PERFORM)
                    {
                        float32_t vmax = BaseCmd.Test_Velocity > 0.01f ? BaseCmd.Test_Velocity : 1.0f;
                        float32_t amax = BaseCmd.Test_Accel    > 0.01f ? BaseCmd.Test_Accel    : 2.0f;
                        if (vmax > 4.0f) vmax = 4.0f;
                        if (amax > 8.0f) amax = 8.0f;

                        switch (test.phase)
                        {
                            case 0: /* going ref+360° — wait done */
                                if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                                {
                                    test.delay_start = HAL_GetTick();
                                    test.phase = 1;
                                }
                                break;

                            case 1: /* 0.5s delay */
                                if (HAL_GetTick() - test.delay_start >= 500)
                                {
                                    test.delay_start = 0;
                                    test.phase = 2;
                                    TRAJ_Plan(PROFILE_SCURVE, ref_deg, vmax, amax, 2.5f * amax);
                                }
                                break;

                            case 2: /* going back to ref — wait done */
                                if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                                {
                                    test.active = 0;
                                    PilotLamp_SetWhite(0);
                                }
                                break;
                        }
                    }
                    else /* PRECISION */
                    {
                        switch (test.phase)
                        {
                            case 0: /* going to Final (go) */
                                if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                                {
                                    test.delay_start = HAL_GetTick();
                                    test.phase = 1;
                                }
                                break;

                            case 1: /* 1s delay after go */
                                if (HAL_GetTick() - test.delay_start >= 1000)
                                {
                                    test.delay_start = 0;
                                    test.phase = 2;
                                    TRAJ_Plan(PROFILE_MINJERK, t_init, TrajManager_DynTime(t_init, Kalman.X[0]), 0.0f, 0.0f);
                                }
                                break;

                            case 2: /* going to Init (back) */
                                if (traj_mgr.state.Complete == 1 && !traj_mgr.running)
                                {
                                    test.delay_start = HAL_GetTick();
                                    test.phase = 3;
                                }
                                break;

                            case 3: /* 1s delay after back — count round trip */
                                if (HAL_GetTick() - test.delay_start >= 1000)
                                {
                                    test.repeat++;
                                    if (test.repeat >= t_repeat_count)
                                    {
                                        test.active = 0;
                                        PilotLamp_SetWhite(0);
                                    }
                                    else
                                    {
                                        test.delay_start = 0;
                                        test.phase = 0;
                                        TRAJ_Plan(PROFILE_MINJERK, t_final, TrajManager_DynTime(t_final, Kalman.X[0]), 0.0f, 0.0f);
                                    }
                                }
                                break;
                        }
                    }
                }
            }   /* end Trust_Basesystem block */

            /* ---- Sequence step-advance (runs regardless of trust mode) ---- */
            if (seq.active)
            {
                    switch (seq.phase)
                    {
                        case 0: /* going to pick — wait trajectory done AND position reached */
                            if (traj_mgr.state.Complete == 1 && !traj_mgr.running
                                && fabsf(Kalman.X[0] - sys_state.cur_pos) <= 0.001745f)
                                seq.phase = 1;

                            break;

                        case 1: /* gripper pick — or 2s delay if gripper disabled */
                        {
                        	/* Start delay timer on first entry */
                            if (robot_gripper.delay_start == 0)
                                robot_gripper.delay_start = HAL_GetTick();

                            uint8_t pick_done = 0;
                            if (BaseCmd.Gripper_Auto_En)
                                pick_done = Gripper_Pick(&robot_gripper);

                            /* Advance after gripper done OR after timeout */
                            if (pick_done || HAL_GetTick() - robot_gripper.delay_start >= GRIPPER_TIMEOUT_MS)
                            {
                                robot_gripper.delay_start  = 0;
                                robot_gripper.action_phase = 0;   /* reset gripper SM */
                                uint8_t   slot     = seq.step * 2 + 1;  /* place slot */
                                int16_t   raw      = (slot < 10) ? BaseCmd.PickPlace_Sequence[slot] : 0;
                                float32_t home_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
                                float32_t tgt      = home_deg + (float)abs(raw) * DEG_PER_HOLE;
                                TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                                seq.phase = 2;
                            }


                            break;
                        }

                        case 2: /* going to place — wait trajectory done AND position reached */
                            if (traj_mgr.state.Complete == 1 && !traj_mgr.running
                                && fabsf(Kalman.X[0] - sys_state.cur_pos) <= 0.001745f)
                                seq.phase = 3;

                            break;

                        case 3: /* gripper place — or 5s timeout if gripper disabled/stuck */
                        {
                            /* Start delay timer on first entry */
                            if (robot_gripper.delay_start == 0)
                                robot_gripper.delay_start = HAL_GetTick();

                            uint8_t place_done = 0;
                            if (BaseCmd.Gripper_Auto_En)
                                place_done = Gripper_Place(&robot_gripper);

                            /* Advance after gripper done OR after timeout */
                            if (place_done || HAL_GetTick() - robot_gripper.delay_start >= GRIPPER_TIMEOUT_MS)
                            {
                                robot_gripper.delay_start  = 0;
                                robot_gripper.action_phase = 0;   /* reset gripper SM */
                                place_done = 1;
                            }

                            if (place_done)
                            {
                                seq.step++;
                                seq.phase = 0;
                                if ((uint16_t)seq.step >= BaseCmd.PickPlace_Pairs)
                                {
                                    /* All pairs done — return to home */
                                    seq.active = 0;
                                    {
                                        float32_t tgt = SYSTEM_STATE_convert_rad2degree(sys_state.home_pos);
                                        TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                                    }
                                }
                                else
                                {
                                    /* Next pair: go to next pick */
                                    uint8_t   slot     = seq.step * 2;   /* pick slot */
                                    int16_t   raw      = (slot < 10) ? BaseCmd.PickPlace_Sequence[slot] : 0;
                                    float32_t home_deg = SYSTEM_STATE_convert_rad2degree(sys_state.basesystem_home_pos);
                                    float32_t tgt      = home_deg + (float)abs(raw) * DEG_PER_HOLE;
                                    TRAJ_Plan(PROFILE_MINJERK, tgt, TrajManager_DynTime(tgt, Kalman.X[0]), 0.0f, 0.0f);
                                }
                            }
                            break;
                        }
                    }
                }
            /* ---- Auto-return to WAITING after move completes (Basesystem) ---- */
            if (sys_state.trust == Trust_Basesystem
                && traj_mgr.state.Complete == 1
                && !traj_mgr.running
                && !seq.active
                && !test.active)
            {
                PilotLamp_SetWhite(0);
                sys_state.cur_state = STATE_WAITING_COMMAND;
            }

            /* ---- Joystick: auto-return to WAITING when move completes ---- */
            if (sys_state.trust == Trust_Joystick
                && traj_mgr.state.Complete == 1 && !traj_mgr.running
                && !seq.active)
            {
                sys_state.cur_state = STATE_WAITING_COMMAND;
            }

            break;

        /* ------------------------------------------------------------------ */
        default:
            sys_state.cur_state = STATE_WAITING_COMMAND;
            break;
    }

    /* Always update edge-detection history regardless of state (raw values) */
    joy_prev.white_left  = robot_joy.raw_btn_white_left;
    joy_prev.white_right = robot_joy.raw_btn_white_right;
    joy_prev.black       = robot_joy.raw_btn_black;
    joy_prev.blue        = robot_joy.raw_btn_blue;
    joy_prev.yellow      = robot_joy.raw_btn_yellow;
    joy_prev.red         = robot_joy.raw_btn_red;
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
        if (BaseCmd.Target_Mode == CMD_MODE_TEST)
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);   /* white lamp blinks on each hole detection in TEST mode */
    }
}

/* =========================================================================
 * Robot_Period_Control_Loop  — 2 kHz ISR
 * ========================================================================= */
void Robot_Period_Control_Loop(void)
{
    /* 1. Encoder (always runs) */
    SYSTEM_STATE_Encoder_Compute(&encoder);

    /* ================================================================
     * 2. Kalman predict + update
     * ================================================================ */
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vout);
    KALMAN_Calc_Acceleration(&Kalman, vout);

    /* 5. Push Kalman estimates into cascade struct */
    Controller.pos_state         = Kalman.X[0];
    Controller.velocity_state    = Kalman.X[1];
    Controller.disturbance_state = Kalman.X[2];

    /* ================================================================
     * 6. Trajectory + Cascade
     *    CASCADE_Compute handles outer PID (÷10), inner PID, both
     *    feedforwards, and writes Controller.voltage_output.
     * ================================================================ */
    if (traj_mgr.running)
    {
        Traj_State_t ref = TrajManager_Step(&traj_mgr);
        CASCADE_Compute(&Controller, ref.pos, ref.velo);

        if (ref.Complete == 1)
            sys_state.cur_pos = Controller.pos_setpoint;   /* latch final pos */
    }
    else
    {
        /* Idle — hold last commanded position */
        CASCADE_Compute(&Controller, sys_state.cur_pos, 0.0f);
    }

    /* 7. Drive motor */
    vout = Controller.voltage_output;
    PWM(vout, 0.4f);   /* 0.5 V offset = static-friction compensation */
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
