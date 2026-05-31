/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LAB4 — Trajectory test with UART streaming to MATLAB
  *                   Control loop only (no BaseSystem / joystick / state machine)
  *
  * Live Expression variables to watch in debugger:
  *   encoder.encoder_rad   — actual position [rad]
  *   Kalman.X[0]           — Kalman position  [rad]
  *   Kalman.X[1]           — Kalman velocity  [rad/s]
  *   traj_mgr.state.Pos    — reference position [rad]
  *   traj_mgr.state.Vel    — reference velocity [rad/s]
  *   traj_mgr.state.Complete — 1 when trajectory finished
  *   vout                  — controller voltage output [V]
  *   traj_start            — write 1 to trigger a new trajectory
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

/* USER CODE BEGIN Includes */
#include "kalman.h"
#include "trajectory.h"
#include "cascade.h"
#include "system_state.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* ---- Controller gains ---- */
#define Kp_inner  1.02259f
#define Ki_inner  10.2259f
#define Kd_inner  0.0f

#define Kp_outer  2.0f
#define Ki_outer  0.0f
#define Kd_outer  2.8f

/* ---- Friction feedforward ---- */
#define static_friction_ff   0.65f
#define dynamic_friction_ff  0.45f
#define friction_velo_thresh 0.01f   /* rad/s — below this = static zone */

/* ---- Trajectory test parameters (edit via Live Expression or recompile) ---- */
#define TRAJ_TEST_PROFILE     TRAJ_PROFILE_SCURVE
#define TRAJ_TEST_TARGET_DEG  90.0f   /* target position [deg]              */
#define TRAJ_TEST_VMAX        4.05f   /* rad/s  (S-curve / min-jerk-vlim)   */
#define TRAJ_TEST_AMAX        4.8f    /* rad/s² (S-curve / trapezoid)       */
#define TRAJ_TEST_JMAX        1.5f    /* rad/s³ (S-curve only)              */
#define TRAJ_TEST_TIME        1.5f    /* s      (min-jerk / trapezoid total) */
#define TRAJ_TEST_ACCT        0.3f    /* s      (trapezoid accel time)      */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* ---- Hardware objects ---- */
Cascade                   Controller;
PID                       Inner;
PID                       Outer;
KALMAN_Multi_Model_Params Kalman;
Feedforward               ff;
Encoder                   encoder;
TrajManager               traj_mgr;
Proximity                 ref_pos;

/* ---- Motor parameters ---- */
MOTOR_PARAMS my_motor = {
    .kt  = 0.20906532f,
    .km  = 0.2316f,
    .J   = 0.107E0f,
    .B   = 0.995f,
    .R   = 0.4821323156f,
    .L   = 0.0002893301219f,
    .tau = 0.05f
};

/* ---- System state ---- */
SYSTEM_STATE sys_state;

/* ---- Controller output ---- */
float32_t vout                = 0.0f;
float32_t friction_feedforward = 0.0f;
float32_t actual_vin           = 0.0f;   /* for friction FF only */
int       moving               = 0;

/* ---- Point-to-Point command struct (edit all fields in Live Expression) ----
 *
 * HOW TO USE:
 *  1. Set p2p.profile   : 0=MinJerk  1=MinJerk_Vlim  2=Trapezoid  3=SCurve
 *  2. Set p2p.target_deg: destination [degrees from current home]
 *  3. Set params for your chosen profile (unused fields are ignored):
 *       MinJerk      -> p2p.time
 *       MinJerk_Vlim -> p2p.vmax, p2p.time (duration cap)
 *       Trapezoid    -> p2p.time (total), p2p.acct (accel time)
 *       SCurve       -> p2p.vmax, p2p.amax, p2p.jmax
 *  4. Write p2p.trigger = 1  →  move fires immediately
 * ========================================================================= */
typedef struct {
    /* --- Trigger --- */
    volatile uint8_t  trigger;      /* write 1 to fire the move              */

    /* --- Profile selection --- */
    volatile int      profile;      /* TrajProfile: 0=MinJerk 1=MJ_Vlim 2=Trap 3=SCurve */

    /* --- Target --- */
    volatile float32_t target_deg; /* destination position [deg]            */

    /* --- Profile parameters --- */
    volatile float32_t vmax;       /* max velocity  [rad/s]   — MJVlim / SCurve        */
    volatile float32_t amax;       /* max accel     [rad/s²]  — SCurve / Trapezoid      */
    volatile float32_t jmax;       /* max jerk      [rad/s³]  — SCurve only             */
    volatile float32_t time;       /* move duration [s]       — MinJerk / Trapezoid     */
    volatile float32_t acct;       /* accel time    [s]       — Trapezoid only          */
} P2P_Cmd;

P2P_Cmd p2p = {
    .trigger    = 0,
    .profile    = TRAJ_TEST_PROFILE,
    .target_deg = TRAJ_TEST_TARGET_DEG,
    .vmax       = TRAJ_TEST_VMAX,
    .amax       = TRAJ_TEST_AMAX,
    .jmax       = TRAJ_TEST_JMAX,
    .time       = TRAJ_TEST_TIME,
    .acct       = TRAJ_TEST_ACCT,
};

/* ---- UART streaming ---- */
/* Packet: [0xFF] + 8×double(64-bit) + [0x0F] = 66 bytes
 *  Index  Field
 *  0      Kalman.X[0]  — Kalman position  [rad]
 *  1      Kalman.X[1]  — Kalman velocity  [rad/s]
 *  2      Kalman.X[2]  — Kalman disturbance current [A]
 *  3      Kalman.X[3]  — Kalman disturbance torque  [Nm]
 *  4      encoder.encoder_degree  — actual position [deg]
 *  5      disturbance_value       — Kalman.X[2] cast for MATLAB convenience
 *  6      traj_mgr.state.Pos      — reference position [rad]
 *  7      traj_mgr.state.Vel      — reference velocity [rad/s]
 */
#define UART_PKT_SIZE  66   /* 1 header + 8*8 data + 1 footer */
static uint8_t  uart_buf[UART_PKT_SIZE];
float32_t disturbance_value = 0.0f;   /* = Kalman.X[2], exposed for Live Expression */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Robot_Period_Control_Loop(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

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

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM16_Init();
    MX_LPUART1_UART_Init();
    MX_FDCAN1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM20_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */
    /* 1. Start hardware timers */
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    PWM(0.0f, 0.0f);

    /* 2. Init encoder & trajectory manager */
    SYSTEM_STATE_Encoder_Init(&htim2, &htim3, &encoder);
    TrajManager_Init(&traj_mgr, &htim2);
    __HAL_TIM_SET_COUNTER(&htim3, 30000);

    /* 3. Init Kalman & cascade controller */
    KALMAN_Multi_Model_Init(&Kalman, 3e-8f, 9.9e-5f, 3e-4f, &my_motor);
    CASCADE_Controller_Init(Kp_inner, Ki_inner, Kd_inner, &Inner, -10.5f, 10.5f, 0.0005f);
    CASCADE_Controller_Init(Kp_outer, Ki_outer, Kd_outer, &Outer, -4.0f,  4.0f, 0.005f);
    CASCADE_Cascade_Start(&Controller, &Inner, &Outer, &ff, &my_motor, &Kalman);

    /* 4. Homing — ISR stays off until homing completes */
    ref_pos.flag_ready      = 0;
    ref_pos.state_detection = 0;
    ref_pos.proximity_flag  = 0;

    sys_state.cur_state      = STATE_WAITING_COMMAND;
    sys_state.cur_pos        = 0.0f;
    sys_state.cur_pos_degree = 0.0f;

    HAL_TIM_RegisterCallback(&htim20, HAL_TIM_PERIOD_ELAPSED_CB_ID, Robot_Period_Control_Loop);
    /* TIM20 started AFTER homing — see while loop */
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* --- Homing (runs until ref_pos.flag_ready == 1, ISR is off) --- */
        if (ref_pos.flag_ready == 0)
        {
            SYSTEM_STATE_Homing(&ref_pos);
            PWM(ref_pos.vout, 0.0f);
        }
        else if (ref_pos.flag_ready == 1)
        {
            /* Homing done: reset encoder counter to center, seed Kalman, start ISR */
            __HAL_TIM_SET_COUNTER(encoder.htim_encoder, ref_pos.reference_counter);
            /* Reset wrap tracking so Encoder_Compute starts clean from new reference */
            encoder.wrap_counter    = 0;
            encoder.encoder_prev_data = (float32_t)ref_pos.reference_counter;
            SYSTEM_STATE_Encoder_Compute(&encoder);
            Kalman.X[0] = encoder.encoder_rad;
            Kalman.X[1] = 0.0f;
            Kalman.X[2] = 0.0f;
            Kalman.X[3] = 0.0f;
            /* Sync cur_pos to actual encoder position so WAITING holds correctly */
            sys_state.cur_pos        = 0;
            sys_state.cur_pos_degree = SYSTEM_STATE_convert_rad2degree(encoder.encoder_rad);
            PWM(0.0f, 0.0f);
            ref_pos.flag_ready = 2;   /* mark done so this block runs only once */
            HAL_TIM_Base_Start_IT(&htim20);
        }

        /* --- P2P trigger: write p2p.trigger = 1 in Live Expression --- */
        if (p2p.trigger)
        {
            p2p.trigger = 0;

            float32_t p1, p2_param, p3;
            switch (p2p.profile)
            {
                case TRAJ_PROFILE_MINJERK:
                    p1 = p2p.time;  p2_param = 0.0f;       p3 = 0.0f;      break;
                case TRAJ_PROFILE_MINJERK_VLIM:
                    p1 = p2p.vmax;  p2_param = p2p.time;   p3 = 0.0f;      break;
                case TRAJ_PROFILE_TRAPEZOID:
                    p1 = p2p.time;  p2_param = p2p.acct;   p3 = 0.0f;      break;
                case TRAJ_PROFILE_SCURVE:
                    p1 = p2p.vmax;  p2_param = p2p.amax;   p3 = p2p.jmax;  break;
                case TRAJ_PROFILE_MINSNAP:
                default:
                    p1 = p2p.time;  p2_param = 0.0f;       p3 = 0.0f;      break;
            }

            /* Snap cur_pos to actual position so WAITING → RUNNING has no step */
            sys_state.cur_pos        = SYSTEM_STATE_convert_degree2rad(p2p.target_deg);
            sys_state.cur_pos_degree = p2p.target_deg;

            TrajManager_Plan(&traj_mgr,
                             p2p.profile,
                             p2p.target_deg,
                             p1, p2_param, p3,
                             encoder.encoder_rad);
            sys_state.cur_state = STATE_RUNNING;
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

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/* USER CODE BEGIN 4 */

/* =========================================================================
 * HAL_GPIO_EXTI_Callback — proximity sensor on PC10 (EXTI line 10)
 * ========================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_10)
    {
        ref_pos.instant_detect = (uint16_t)__HAL_TIM_GET_COUNTER(encoder.htim_encoder);
        ref_pos.proximity_flag = 1;
    }
}

/* =========================================================================
 * Robot_Period_Control_Loop  — called by TIM20 period elapsed ISR (0.5 ms)
 * ========================================================================= */
void Robot_Period_Control_Loop(TIM_HandleTypeDef *htim)
{
    (void)htim;

    /* 1. Read encoder */
    SYSTEM_STATE_Encoder_Compute(&encoder);

    /* 2. Step Kalman filter */
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vout);

    /* 3. State machine */
    Traj_State_t ref;

    switch (sys_state.cur_state)
    {
        case STATE_RUNNING:
            /* Advance trajectory and follow it */
            ref = TrajManager_Step(&traj_mgr);

            /* When trajectory finishes, latch cur_pos and go to WAITING */
            if (traj_mgr.state.Complete && !traj_mgr.running)
            {
                sys_state.cur_pos        = traj_mgr.cur_pos_rad;
                sys_state.cur_pos_degree = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos);
                sys_state.cur_state      = STATE_WAITING_COMMAND;
            }
            break;

        case STATE_WAITING_COMMAND:
        default:
            /* Hold position at cur_pos, zero velocity override */
            ref.pos      = sys_state.cur_pos;
            ref.velo     = 0.0f;
            ref.accel    = 0.0f;
            ref.Complete = 1;
            break;
    }

    /* 4. Feed Kalman states into cascade struct (same pattern as Studio_Code) */
    Controller.pos_state         = Kalman.X[0];
    Controller.velocity_state    = Kalman.X[1];
    Controller.current_state     = Kalman.X[3];
    Controller.disturbance_state = Kalman.X[2];

    /* 5. Cascade control */
    CASCADE_Compute(&Controller, ref.pos, ref.velo);

    /* 6. Friction feedforward — same values as Studio_Code */
    float32_t vel = Kalman.X[1];
    if (fabsf(vel) < friction_velo_thresh)
        /* Static zone: apply in direction of position error */
        friction_feedforward = static_friction_ff
                               ;
    else
        /* Dynamic zone: apply in direction of motion */
        friction_feedforward = dynamic_friction_ff ;

    actual_vin = Controller.voltage_output;   /* pre-friction, for reference */
    vout = Controller.voltage_output;

    /* 5. Apply PWM */
    PWM(vout, friction_feedforward);

    /* 6. Update convenience variable */
    disturbance_value = Kalman.X[2];

    /* 7. UART packet to MATLAB — send every 5 ticks (400 Hz) to avoid DMA collisions */
    static uint8_t uart_div = 0;
    if (++uart_div >= 5) uart_div = 0;

    if (uart_div == 0 && hlpuart1.gState == HAL_UART_STATE_READY)
    {
        uart_buf[0] = 0xFF;
        double fields[8] = {
            (double)Kalman.X[0],              /* 0: Kalman position     [rad]  */
            (double)Kalman.X[1],              /* 1: Kalman velocity     [rad/s]*/
            (double)Kalman.X[2],              /* 2: Kalman disturbance  [A]    */
            (double)sys_state.cur_state,              /* 3: Kalman dist. torque [Nm]   */
            (double)encoder.encoder_rad,   /* 4: actual position     [deg]  */
            (double)disturbance_value,        /* 5: disturbance value   [A]    */
            (double)ref.pos,                  /* 6: trajectory position [rad]  */
            (double)ref.velo,                 /* 7: trajectory velocity [rad/s]*/
        };
        memcpy(&uart_buf[1], fields, 64);
        uart_buf[65] = 0x0F;
        HAL_UART_Transmit_DMA(&hlpuart1, uart_buf, UART_PKT_SIZE);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
