/* ==========================================================================
 *  LAB_Manager.c
 *
 *  Owns all control-system objects, lab parameters, and the 2 kHz ISR.
 *  main.c only calls LAB_Init() and LAB_MainLoop().
 *
 *  Lab selection (lab.lab_select):
 *    1 = Kalman filter test   — open-loop voltage, dual Kalman (with/no input)
 *    2 = Inner-loop PID       — velocity setpoint, optional FF logging
 *    3 = Cascade control      — full trajectory + hold position after end
 *    4 = Dead band ramp       — 0 → 9 V at 0.5 V/s, then back to 0
 * ========================================================================== */

#include "LAB_Manager.h"
#include "main.h"
#include "lab_config.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"
#include <string.h>
#include <math.h>

/* =========================================================================
 * Internal constants
 * ======================================================================= */
#define UART_PKT_SIZE          90u       /* [0xFF] + 11×8 bytes + [0x0F]   */

#define HOLD_POS_DEADZONE_DEG  0.10f     /* outer PID off + integral bleed  */
#define HOLD_INTEGRAL_DECAY    0.95f     /* bleed factor per tick           */

#define RAMP_SLOPE  (0.5f / 2000.0f)    /* 0.5 V/s @ 2 kHz = 0.00025 V/tick */
#define RAMP_MAX     9.0f

#define LAB3_HOLD_MS   500U             /* hold position for this long after trajectory [ms] */
#define LAB3_RESET_MS 2000U             /* auto-reset this long after trajectory ends   [ms] */

/* ---- LAB5 sweep parameters -------------------------------------------- */
#define LAB5_REPEATS      5             /* repetitions per target angle                     */
#define LAB5_STEP_DEG     5.0f          /* step between targets [deg]                       */
#define LAB5_N_TARGETS    72            /* 5° .. 360° → 72 targets                          */
#define LAB5_BREAK_MS     2500U         /* idle pause after each move (PWM already off)     */

typedef enum {
    L5_IDLE = 0,
    L5_WAIT_FWD,        /* forward move running — waiting for lab3_done      */
    L5_BREAK_FWD,       /* 1 s idle after forward done, before return home   */
    L5_WAIT_HOME,       /* return-to-home running — waiting for lab3_done    */
    L5_BREAK_HOME,      /* 1 s idle after home done, before next forward     */
    L5_DONE
} LAB5_Phase_t;

/* Friction thresholds */
#define FRICTION_VELO_THRESH   0.07f    /* rad/s — tanh roll-off width     */
#define FRICTION_POS_THRESH    0.0087f  /* rad   (~0.5 deg)                */

#define Disturbance_Scale      CFG_DISTURBANCE_SCALE

/* =========================================================================
 * Hardware objects
 * ======================================================================= */
static Cascade                   Controller;
static PID                       Inner;
static PID                       Outer;
static Feedforward               ff;
static FrictionFF                friction;
static TrajManager               traj_mgr;
static Traj_State_t              ref;

KALMAN_Multi_Model_Params        Kalman;          /* LAB1/2/3 — main filter    */
KALMAN_Multi_Model_Params        Kalman_NoInput;  /* LAB1     — no-input filter */
Encoder                          encoder;
Proximity                        ref_pos;
SYSTEM_STATE                     sys_state;

static MOTOR_PARAMS my_motor = {
    .kt       = 2.2461f,
    .km       = 2.2461f,
    .J        = 8.6701E-1f,
    .B        = 0.4308f,
    .R        = 0.4821323156f,
    .L        = 0.0002893301219f,
    .tau      = 0.05f,
    .tau_ref  = CFG_TAU_REF,
    .tau_dist = CFG_TAU_DIST,
};

/* =========================================================================
 * Lab parameters  (written live from MATLAB / Live Expression)
 * ======================================================================= */
LAB_Params lab = {
    .reset              = 0,
    .lab_select         = CFG_LAB_SELECT,
    .q_process          = CFG_Q_PROCESS,
    .q_disturbance      = CFG_Q_DISTURBANCE,
    .r_measurement      = CFG_R_MEASUREMENT,
    .direct_voltage     = CFG_DIRECT_VOLTAGE,
    .Kp_in  = CFG_KP_IN,  .Ki_in  = CFG_KI_IN,  .Kd_in  = CFG_KD_IN,
    .Kp_out = CFG_KP_OUT, .Ki_out = CFG_KI_OUT, .Kd_out = CFG_KD_OUT,
    .tau_dist           = CFG_TAU_DIST,
    .tau_ref            = CFG_TAU_REF,
    .en_disturbance_ff  = CFG_EN_DISTURBANCE_FF,
    .en_reference_ff    = CFG_EN_REFERENCE_FF,
    .en_pid             = CFG_EN_PID,
    .en_integral_windup = CFG_EN_INTEGRAL_WINDUP,
    .velo_setpoint      = CFG_VELO_SETPOINT,
    .pos_setpoint_deg   = CFG_POS_SETPOINT_DEG,
    .traj_select        = CFG_TRAJ_SELECT,
    .trap_vmax          = CFG_TRAP_VMAX,   .trap_amax    = CFG_TRAP_AMAX,
    .scurve_vmax        = CFG_SCURVE_VMAX, .scurve_amax  = CFG_SCURVE_AMAX,
    .scurve_jmax        = CFG_SCURVE_JMAX,
    .friction_static    = CFG_FRICTION_STATIC,
    .friction_dynamic   = CFG_FRICTION_DYNAMIC,
    .friction_en        = CFG_EN_FRICTION,
    .disturbance_scale  = Disturbance_Scale,
    .lab5_active        = 0,
    .lab5_all_done      = 0
};

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

/* =========================================================================
 * Runtime state
 * ======================================================================= */
volatile uint8_t   lab3_done          = 0;
         uint8_t   home_here          = 1;

static volatile uint8_t    lab3_holding    = 0;    /* 1 = in 0.5 s post-traj hold window */
static volatile uint32_t   lab3_hold_tick  = 0;    /* HAL_GetTick() when hold started     */
static volatile uint32_t   lab3_reset_tick = 0;    /* HAL_GetTick() when trajectory ended */
static volatile uint8_t    lab3_stepped    = 0;    /* 1 = target already incremented this move */

/* LAB5 sequencer state */
static LAB5_Phase_t        lab5_phase     = L5_IDLE;
static uint8_t             lab5_step      = 0;   /* 0..71  → target = (step+1)*5°  */
static uint8_t             lab5_repeat    = 0;   /* 0..4                            */
static uint32_t            lab5_break_tick = 0;  /* HAL_GetTick() snapshot for break */
/* lab.lab5_active / lab.lab5_all_done live in lab struct (accessible via Live Expression) */

static volatile uint8_t    ramp_active = 0;
static volatile uint8_t    ramp_up     = 1;
static volatile float32_t  ramp_vout   = 0.0f;

static float32_t  vout                = 0.0f;
static float32_t  vin_kalman          = 0.0f;   /* PID + ref_FF only — fed to Kalman */
static float32_t  friction_feedforward = 0.0f;
         float32_t  actual_error       = 0.0f;

static uint8_t    uart_buf[UART_PKT_SIZE];

/* Forward declarations (static helpers defined later in file) */
static void LAB5_Process(void);

/* =========================================================================
 * LAB_Init
 * ======================================================================= */
void LAB_Init(void)
{
    /* System state — encoder, proximity, state machine */
    SYSTEM_STATE_Init(&sys_state, &encoder, &ref_pos, &htim2, &htim3, &htim1);

    /* Trajectory manager */
    TrajManager_Init(&traj_mgr, &htim2);

    /* Kalman filters */
    KALMAN_Multi_Model_Init(&Kalman,        lab.q_process, lab.q_disturbance, lab.r_measurement, &my_motor);
    KALMAN_Multi_Model_Init(&Kalman_NoInput, lab.q_process, lab.q_disturbance, lab.r_measurement, &my_motor);

    /* Cascade controller */
    CASCADE_Controller_Init(lab.Kp_in,  lab.Ki_in,  lab.Kd_in,  &Inner, -10.5f, 10.5f, 0.0005f);
    CASCADE_Controller_Init(lab.Kp_out, lab.Ki_out, lab.Kd_out, &Outer,  -4.0f,  4.0f, 0.002f);
    CASCADE_Cascade_Start(&Controller, &Inner, &Outer, &ff, &my_motor, &Kalman);
    CASCADE_Friction_Init(&friction,
                          lab.friction_static, lab.friction_dynamic,
                          FRICTION_VELO_THRESH, FRICTION_POS_THRESH);

    /* Register 2 kHz control loop with TIM20 */
    HAL_TIM_RegisterCallback(&htim20, HAL_TIM_PERIOD_ELAPSED_CB_ID, Robot_Period_Control_Loop);

    /* Preset encoder counter (no homing sweep for LAB) */
    __HAL_TIM_SET_COUNTER(encoder.htim_encoder, 30000);
}

/* =========================================================================
 * LAB_HandleReset  — called when lab.reset is written to 1
 * ======================================================================= */
static void LAB_HandleReset(void)
{
    lab.reset           = 0;
    lab3_done           = 0;
    lab3_holding        = 0;
    lab3_stepped        = 0;
    p2p.trigger         = 0;
    sys_state.cur_state = STATE_WAITING_COMMAND;
    __HAL_TIM_SET_COUNTER(encoder.htim_encoder, 30000);
    ref_pos.flag_ready  = 0;   /* force SetHomeHere to re-run */

    /* Sync live-tuned params → hardware structs */
    Inner.kp = lab.Kp_in;  Inner.ki = lab.Ki_in;  Inner.kd = lab.Kd_in;
    Outer.kp = lab.Kp_out; Outer.ki = lab.Ki_out; Outer.kd = lab.Kd_out;

    Kalman.Q[5]          = lab.q_process;
    Kalman.Q[10]         = lab.q_disturbance;
    Kalman_NoInput.Q[5]  = lab.q_process;
    Kalman_NoInput.Q[10] = lab.q_disturbance;
    Kalman.R[0]          = lab.r_measurement;
    Kalman_NoInput.R[0]  = lab.r_measurement;

    my_motor.tau_dist    = lab.tau_dist;
    my_motor.tau_ref     = lab.tau_ref;
    CASCADE_Feedforward_Init(&my_motor, &ff);

    CASCADE_Friction_Init(&friction,
                          lab.friction_static, lab.friction_dynamic,
                          FRICTION_VELO_THRESH, FRICTION_POS_THRESH);

    traj_mgr.state.Complete = 1;
    traj_mgr.running        = 0;
}

/* =========================================================================
 * LAB_HandleTrigger  — called when p2p.trigger is written to 1
 * ======================================================================= */
static void LAB_HandleTrigger(void)
{
    p2p.trigger  = 0;
    lab3_done    = 0;
    lab3_holding = 0;
    lab3_stepped = 0;

    /* LAB4 — dead band ramp mode */
    if (lab.lab_select == 4)
    {
        ramp_vout   = 0.0f;
        ramp_up     = 1;
        ramp_active = 1;
        return;
    }

    /* LAB3 — plan trajectory */
    float32_t p1, p2_param, p3;
    switch (p2p.profile)
    {
        case TRAJ_PROFILE_MINJERK:
            p1 = p2p.time;  p2_param = 0.0f;      p3 = 0.0f;      break;
        case TRAJ_PROFILE_MINJERK_VLIM:
            p1 = p2p.vmax;  p2_param = p2p.time;  p3 = 0.0f;      break;
        case TRAJ_PROFILE_TRAPEZOID:
            p1 = p2p.time;  p2_param = p2p.acct;  p3 = 0.0f;      break;
        case TRAJ_PROFILE_SCURVE:
            p1 = p2p.vmax;  p2_param = p2p.amax;  p3 = p2p.jmax;  break;
        case TRAJ_PROFILE_MINSNAP:
        default:
            p1 = p2p.time;  p2_param = 0.0f;      p3 = 0.0f;      break;
    }

    sys_state.cur_pos        = SYSTEM_STATE_convert_degree2rad(p2p.target_deg);
    sys_state.cur_pos_degree = p2p.target_deg;

    TrajManager_Plan(&traj_mgr,
                     p2p.profile,
                     p2p.target_deg,
                     p1, p2_param, p3,
                     encoder.encoder_rad);
    sys_state.cur_state = STATE_RUNNING;
}

/* =========================================================================
 * LAB_MainLoop  — call repeatedly from main() while(1)
 * ======================================================================= */
void LAB_MainLoop(void)
{
    if (lab.reset)
        LAB_HandleReset();

    if (home_here && ref_pos.flag_ready != 2)
        SYSTEM_STATE_SetHomeHere(&ref_pos, &encoder, &Kalman, &sys_state, &htim20);

    /* LAB5 sequencer — must run before trigger check so trigger set here
       is caught by LAB_HandleTrigger in the same main-loop pass */
    if (lab.lab_select == 5)
        LAB5_Process();

    if (p2p.trigger)
        LAB_HandleTrigger();
}

/* =========================================================================
 * LAB_BuildTxPacket  — fill uart_buf fields for the active lab
 * ======================================================================= */
static void LAB_BuildTxPacket(void)
{
    uart_buf[0] = 0xFF;
    double fields[11] = {0.0};

    switch (lab.lab_select)
    {
    case 1:   /* LAB1 — Kalman filter */
        fields[0]  = (double)Kalman.X[0];
        fields[1]  = (double)Kalman.X[1];
        fields[2]  = (double)Kalman.X[2];
        fields[3]  = (double)Kalman.X[3];
        fields[4]  = (double)Kalman_NoInput.X[0];
        fields[5]  = (double)Kalman_NoInput.X[1];
        fields[6]  = (double)Kalman_NoInput.X[2];
        fields[7]  = (double)Kalman_NoInput.X[3];
        fields[8]  = (double)encoder.encoder_rad;
        fields[9]  = (double)Kalman.X[1];
        fields[10] = (double)vout;
        break;

    case 2:   /* LAB2 — inner loop */
        fields[0]  = (double)Kalman.X[0];
        fields[1]  = (double)Kalman.X[1];
        fields[2]  = (double)Kalman.X[2];
        fields[3]  = (double)Kalman.X[3];
        fields[4]  = (double)lab.velo_setpoint;
        fields[5]  = (double)Controller.feedforward->disturbance_value;
        fields[6]  = (double)friction_feedforward;
        fields[7]  = (double)encoder.encoder_rad;
        fields[8]  = (double)Kalman.X[1];
        fields[9]  = (double)Controller.feedforward->reference_value;
        fields[10] = 0.0;
        break;

    case 3:   /* LAB3 — cascade */
        fields[0]  = (double)(vout_applied * Kalman.X[3]);  /* power [W] — uses post-saturation voltage */
        fields[1]  = (double)Kalman.X[1];
        fields[2]  = (double)Kalman.X[2];
        fields[3]  = (double)friction_feedforward;
        fields[4]  = (double)ref.pos;
        fields[5]  = (double)ref.velo;
        fields[6]  = (double)vout;
        fields[7]  = (double)encoder.encoder_rad;
        fields[8]  = (double)Controller.feedforward->reference_value;
        fields[9]  = (double)Controller.feedforward->disturbance_value;
        fields[10] = (double)traj_mgr.state.Complete;
        break;

    case 4:   /* LAB4 — dead band ramp */
        fields[0]  = (double)encoder.encoder_rad;
        fields[1]  = (double)Kalman.X[1];
        fields[2]  = (double)vout;
        fields[3]  = (double)ramp_active;
        break;

    case 5:   /* LAB5 — auto sweep (same signals as LAB3 + sweep status) */
        fields[0]  = (double)(vout_applied * Kalman.X[3]);  /* power [W] — post-saturation */
        fields[1]  = (double)Kalman.X[1];
        fields[2]  = (double)Kalman.X[2];
        fields[3]  = (double)friction_feedforward;
        fields[4]  = (double)ref.pos;
        fields[5]  = (double)ref.velo;
        fields[6]  = (double)vout;
        fields[7]  = (double)encoder.encoder_rad;
        fields[8]  = (double)Controller.feedforward->reference_value;
        fields[9]  = (double)Controller.feedforward->disturbance_value;
        fields[10] = (double)traj_mgr.state.Complete;
        break;

    default:
        break;
    }

    memcpy(&uart_buf[1], fields, 88);
    uart_buf[89] = 0x0F;
    HAL_UART_Transmit_DMA(&hlpuart1, uart_buf, UART_PKT_SIZE);
}

/* =========================================================================
 * LAB1_Run  — open-loop voltage, dual Kalman
 * ======================================================================= */
static void LAB1_Run(void)
{
    KALMAN_Multi_Model_Compute(&Kalman,         encoder.encoder_rad, lab.direct_voltage);
    KALMAN_Multi_Model_Compute(&Kalman_NoInput, encoder.encoder_rad, 0.0f);

    vout = lab.direct_voltage;
    PWM(vout, 0.0f);
}

/* =========================================================================
 * LAB2_Run  — inner-loop velocity PID + optional FF
 * ======================================================================= */
static void LAB2_Run(void)
{
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vin_kalman);

    CASCADE_Update(&Controller,
                   encoder.encoder_rad, Kalman.X[1],
                   Kalman.X[3], Kalman.X[2],
                   encoder.encoder_rad);

    CASCADE_Disturbance_Compute(&Controller, Kalman.X[2]);
    CASCADE_Ref_Compute(&Controller, lab.velo_setpoint);
    friction_feedforward = CASCADE_Friction_Compute(&friction, Kalman.X[1], 0.0f);

    if (lab.en_pid)
    {
        CASCADE_Controller_Compute_Velocity(&Inner, lab.velo_setpoint, Kalman.X[1]);
        Inner.out = 0;
        /* vin_kalman = PID + ref_FF only */
        vin_kalman = Inner.out + (lab.en_reference_ff ? Controller.feedforward->reference_value : 0.0f);
        /* dist_FF added to motor output only */
        vout = vin_kalman + (lab.en_disturbance_ff ? Controller.feedforward->disturbance_value : 0.0f);
    }
    else
    {
        vin_kalman = 0.0f;
        vout       = 0.0f;
    }

    PWM(vout, lab.friction_en ? friction_feedforward : 0.0f);
}

/* =========================================================================
 * LAB3_HoldPosition  — cascade hold after trajectory ends
 * ======================================================================= */
static void LAB3_HoldPosition(void)
{
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vin_kalman);

    CASCADE_Update(&Controller,
                   encoder.encoder_rad, Kalman.X[1],
                   Kalman.X[3], Kalman.X[2],
                   encoder.encoder_rad);

    if (lab.en_pid)
    {
        float32_t err_deg = fabsf((sys_state.cur_pos - Kalman.X[0]) * 57.295f);

        /* Outer PID — decimated every 4 ticks to match CASCADE_Compute rate */
        Controller.loop_counter++;
        if (Controller.loop_counter >= 4)
        {
            Controller.loop_counter = 0;
            if (err_deg > HOLD_POS_DEADZONE_DEG)
            {
                Outer.ki = lab.Ki_out;
                CASCADE_Controller_Compute_Position(&Outer, sys_state.cur_pos, encoder.encoder_rad);
            }
            else
            {
                Outer.out = 0.0f;
            }
        }
        /* Integral bleed every tick */
        if (err_deg <= HOLD_POS_DEADZONE_DEG)
        {
            Outer.integral *= HOLD_INTEGRAL_DECAY;
            Inner.integral *= HOLD_INTEGRAL_DECAY;
        }

        CASCADE_Controller_Compute_Velocity(&Inner, Outer.out, Kalman.X[1]);
        CASCADE_Ref_Compute(&Controller, Outer.out);
        CASCADE_Disturbance_Compute(&Controller, Kalman.X[2]);

        /* Decay disturbance FF when settled — prevents limit cycle at rest */
        if (fabsf((sys_state.cur_pos - Kalman.X[0]) * 57.295f) < 0.1f)
        {
            Controller.feedforward->disturbance_value *= 0.9f;
            Controller.feedforward->dist_y_prev       *= 0.9f;
            Controller.feedforward->dist_u_prev       *= 0.9f;
        }

        /* vin_kalman = PID + ref_FF (no dist_FF, no friction) */
        vin_kalman = Inner.out
                   + (lab.en_reference_ff ? Controller.feedforward->reference_value : 0.0f);
        /* vout adds dist_FF on top — goes to PWM, not Kalman */
        Controller.feedforward->disturbance_value *= lab.disturbance_scale; 
        vout = vin_kalman
             + (lab.en_disturbance_ff ? Controller.feedforward->disturbance_value : 0.0f);

        if      (vout >  12.0f) vout =  12.0f;
        else if (vout < -12.0f) vout = -12.0f;
    }
    else
    {
        vout = 0.0f;
    }
    friction.dynamic_ff = lab.friction_dynamic;
    friction_feedforward = CASCADE_Friction_Compute(
        &friction, Kalman.X[1], sys_state.cur_pos - encoder.encoder_rad);

    PWM(vout, lab.friction_en ? friction_feedforward : 0.0f);
}

/* =========================================================================
 * LAB3_Run  — cascade trajectory + hold position
 * ======================================================================= */
static void LAB3_Run(void)
{
    /* --- Permanently done: PWM cut, step target once after 2 s ---------- */
    if (lab3_done)
    {
        PWM(0.0f, 0.0f); vout = 0.0f; vin_kalman = 0.0f;
        if (!lab3_stepped && (HAL_GetTick() - lab3_reset_tick >= LAB3_RESET_MS))
        {
            p2p.target_deg += 5.0f;   /* advance to next hole — once only */
            lab3_stepped    = 1;
        }
        return;
    }

    /* --- Post-trajectory hold window (0.5 s) ----------------------------- */
    if (lab3_holding)
    {
        if (HAL_GetTick() - lab3_hold_tick >= LAB3_HOLD_MS)
        {
            /* Hold window expired -> cut PWM, signal done */
            lab3_holding = 0;
            lab3_done    = 1;
            PWM(0.0f, 0.0f);
            vout       = 0.0f;
            vin_kalman = 0.0f;
            return;
        }
        /* Still inside hold window -> run hold position controller */
        LAB3_HoldPosition();
        return;
    }

    /* --- Normal trajectory execution ------------------------------------- */
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vin_kalman);

    Outer.ki = lab.Ki_out;
    CASCADE_Update(&Controller,
                   encoder.encoder_rad, Kalman.X[1],
                   Kalman.X[3], Kalman.X[2],
                   encoder.encoder_rad);

    switch (sys_state.cur_state)
    {
        case STATE_RUNNING:
            ref = TrajManager_Step(&traj_mgr);
            if (traj_mgr.state.Complete && !traj_mgr.running)
            {
                /* Trajectory just finished -> start 0.5 s hold window */
                sys_state.cur_pos        = traj_mgr.cur_pos_rad;
                sys_state.cur_pos_degree = SYSTEM_STATE_convert_rad2degree(sys_state.cur_pos);
                lab3_holding   = 1;
                lab3_hold_tick = HAL_GetTick();
                lab3_reset_tick = lab3_hold_tick;   /* 2 s auto-reset starts now */
                PWM(0.0f, 0.0f);
                vout = 0.0f;
                return;
            }
            break;

        case STATE_WAITING_COMMAND:
        default:
            ref.pos      = sys_state.cur_pos;
            ref.velo     = 0.0f;
            ref.accel    = 0.0f;
            ref.Complete = 1;
            break;
    }

    Controller.traj_running = ref.Complete;

    if (lab.en_pid)
        CASCADE_Compute(&Controller, ref.pos, ref.velo);
    friction.dynamic_ff = lab.friction_dynamic;
    friction_feedforward = CASCADE_Friction_Compute(
        &friction, Kalman.X[1], ref.pos - Kalman.X[0]);

    if (lab.en_pid)
    {
        /* vin_kalman = inner PID + ref_FF only */
        vin_kalman = Inner.out + Controller.feedforward->reference_value;
        /* vout = vin_kalman + dist_FF (goes to motor, dist_FF bypasses Kalman) */
        vout = vin_kalman
             + (lab.en_disturbance_ff ? Controller.feedforward->disturbance_value : 0.0f);
    }
    else
    {
        vin_kalman = 0.0f;
        vout       = 0.0f;
    }
    PWM(vout, lab.friction_en ? friction_feedforward : 0.0f);
}

/* =========================================================================
 * LAB5_Process  — auto P2P sweep sequencer (call from LAB_MainLoop)
 *
 *   Sequence per target angle T (deg):
 *     1. Trigger move to T°            (ISR runs LAB3_Run)
 *     2. Wait for lab3_done == 1       (trajectory + 0.5 s hold done)
 *     3. Trigger return to 0°
 *     4. Wait for lab3_done == 1
 *     5. Reset encoder to exact 0      (eliminate cumulative error)
 *     6. Advance repeat / step counter
 *     7. Repeat LAB5_REPEATS times per target, then move to next target
 * ======================================================================= */
static void LAB5_Process(void)
{
    if (lab.lab_select != 5 || !lab.lab5_active) return;

    switch (lab5_phase)
    {
    /* ---- First call after lab.lab5_active = 1 ----------------------------- */
    case L5_IDLE:
        lab5_step      = 0;
        lab5_repeat    = 0;
        lab.lab5_all_done  = 0;
        /* Trigger first forward move */
        p2p.target_deg = (float)(lab5_step + 1) * LAB5_STEP_DEG;
        p2p.trigger    = 1;          /* LAB_HandleTrigger fires this iteration */
        lab5_phase     = L5_WAIT_FWD;
        break;

    /* ---- Waiting for forward move (0° → target) to finish ------------- */
    case L5_WAIT_FWD:
        if (lab3_done)
        {
            /* Forward done — start 1 s break before return */
            lab5_break_tick = HAL_GetTick();
            lab5_phase      = L5_BREAK_FWD;
        }
        break;

    /* ---- 1 s idle after forward move ---------------------------------- */
    case L5_BREAK_FWD:
        if (HAL_GetTick() - lab5_break_tick >= LAB5_BREAK_MS)
        {
            /* Break over — trigger return to home */
            p2p.target_deg = 0.0f;
            p2p.trigger    = 1;
            lab5_phase     = L5_WAIT_HOME;
        }
        break;

    /* ---- Waiting for return move (target → 0°) to finish -------------- */
    case L5_WAIT_HOME:
        if (lab3_done)
        {
            /* Return done — start 1 s break before next forward */
            lab5_break_tick = HAL_GetTick();
            lab5_phase      = L5_BREAK_HOME;
        }
        break;

    /* ---- 1 s idle after home move ------------------------------------- */
    case L5_BREAK_HOME:
        if (HAL_GetTick() - lab5_break_tick >= LAB5_BREAK_MS)
        {
            /* Reset encoder to exact 0 — eliminate cumulative position error.
             * Disable ISR first so encoder_prev_data and the counter are updated
             * atomically — prevents the 2kHz ISR from seeing a stale prev_data
             * against a freshly reset counter (which would compute a huge delta
             * and potentially freeze the encoder reading). */
            __disable_irq();
            __HAL_TIM_SET_COUNTER(encoder.htim_encoder, 30000);
            encoder.wrap_counter      = 0;
            encoder.encoder_prev_data = 0.0f;
            encoder.encoder_rad       = 0.0f;
            encoder.encoder_degree    = 0.0f;
            Kalman.X[0]              = 0.0f;
            Kalman.X[1]              = 0.0f;
            Kalman.X[2]              = 0.0f;
            sys_state.cur_pos        = 0.0f;
            sys_state.cur_pos_degree = 0.0f;
            __enable_irq();

            /* Advance repeat / target counters */
            lab5_repeat++;
            if (lab5_repeat >= LAB5_REPEATS)
            {
                lab5_repeat = 0;
                lab5_step++;
                if (lab5_step >= LAB5_N_TARGETS)
                {
                    lab5_phase        = L5_DONE;
                    lab.lab5_active   = 0;
                    lab.lab5_all_done = 1;
                    break;
                }
            }

            /* Trigger next forward move */
            p2p.target_deg = (float)(lab5_step + 1) * LAB5_STEP_DEG;
            p2p.trigger    = 1;
            lab5_phase     = L5_WAIT_FWD;
        }
        break;

    case L5_DONE:
        /* Sequence complete — wait for MATLAB to detect lab.lab5_all_done */
        break;

    default:
        break;
    }
}

/* =========================================================================
 * LAB4_Run  — dead band ramp 0 → 9V → 0
 * ======================================================================= */
static void LAB4_Run(void)
{
    KALMAN_Multi_Model_Compute(&Kalman, encoder.encoder_rad, vin_kalman);

    if (!ramp_active) { PWM(0.0f, 0.0f); vout = 0.0f; return; }

    if (ramp_up)
    {
        ramp_vout += RAMP_SLOPE;
        if (ramp_vout >= RAMP_MAX) { ramp_vout = RAMP_MAX; ramp_up = 0; }
    }
    else
    {
        ramp_vout -= RAMP_SLOPE;
        if (ramp_vout <= 0.0f)
        {
            ramp_vout   = 0.0f;
            ramp_active = 0;
            lab3_done   = 1;
        }
    }

    vout = ramp_vout;
    PWM(vout, 0.0f);
}

/* =========================================================================
 * Robot_Period_Control_Loop  — TIM20 period-elapsed ISR (2 kHz / 0.5 ms)
 * ======================================================================= */
void Robot_Period_Control_Loop(TIM_HandleTypeDef *htim)
{
    (void)htim;

    /* 1. Update encoder */
    SYSTEM_STATE_Encoder_Compute(&encoder);

    /* 2. Per-lab control — each function runs its own Kalman update first,
     *    then computes control output and applies PWM.
     *    vin_kalman carries the previous tick's PID+ref_FF voltage so the
     *    observer model is always fed what was actually applied. */
    switch (lab.lab_select)
    {
        case 1:  LAB1_Run(); break;
        case 2:  LAB2_Run(); break;
        case 3:  LAB3_Run(); break;
        case 4:  LAB4_Run(); break;
        case 5:  LAB3_Run(); break;   /* LAB5 reuses LAB3 trajectory + hold */
        default: PWM(0.0f, 0.0f); vout = 0.0f; break;
    }

    /* 4. Convenience variable */
    actual_error = fabsf(encoder.encoder_rad - sys_state.cur_pos) * 57.295f;

    /* 5. UART TX — 11 doubles to MATLAB every tick */
    if (hlpuart1.gState == HAL_UART_STATE_READY)
        LAB_BuildTxPacket();
}

/* =========================================================================
 * HAL_GPIO_EXTI_Callback  — proximity sensor on PC10 (EXTI line 10)
 * ======================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        ref_pos.instant_detect = (uint16_t)__HAL_TIM_GET_COUNTER(encoder.htim_encoder);
        ref_pos.proximity_flag = 1;
    }
}
