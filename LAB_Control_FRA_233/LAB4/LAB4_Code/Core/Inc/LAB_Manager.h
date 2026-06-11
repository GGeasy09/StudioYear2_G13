#ifndef LAB_MANAGER_H
#define LAB_MANAGER_H

#include "arm_math.h"
#include "cascade.h"
#include "kalman.h"
#include "system_state.h"
#include "trajectory.h"

/* =========================================================================
 * P2P_Cmd — point-to-point trajectory command
 *   Write from Live Expression / MATLAB to fire a move.
 * ======================================================================= */
typedef struct {
    volatile uint8_t    trigger;      /* write 1 to fire the move              */
    volatile int        profile;      /* TrajProfile: 0=MinJerk 1=MJVlim 2=Trap 3=SCurve */
    volatile float32_t  target_deg;   /* destination position [deg]            */
    volatile float32_t  vmax;         /* max velocity  [rad/s]   — MJVlim / SCurve  */
    volatile float32_t  amax;         /* max accel     [rad/s²]  — SCurve / Trap    */
    volatile float32_t  jmax;         /* max jerk      [rad/s³]  — SCurve only      */
    volatile float32_t  time;         /* move duration [s]       — MinJerk / Trap   */
    volatile float32_t  acct;         /* accel time    [s]       — Trapezoid only   */
} P2P_Cmd;

/* =========================================================================
 * LAB_Params — universal parameter package
 *   Written by MATLAB via CMD 0x10 or Live Expression.
 * ======================================================================= */
typedef struct {
    /* Common */
    volatile uint8_t    reset;              /* 1 = re-home + clear lab3_done        */
    volatile uint8_t    lab_select;         /* 1=Kalman  2=Inner  3=Cascade  4=Dead */
    volatile float32_t  q_process;
    volatile float32_t  q_disturbance;
    volatile float32_t  r_measurement;
    /* LAB1 */
    volatile float32_t  direct_voltage;     /* open-loop voltage [V]                */
    /* LAB2 & LAB3 */
    volatile float32_t  Kp_in,  Ki_in,  Kd_in;
    volatile float32_t  Kp_out, Ki_out, Kd_out;
    volatile float32_t  tau_dist, tau_ref;
    volatile uint8_t    en_disturbance_ff;
    volatile uint8_t    en_reference_ff;
    volatile uint8_t    en_pid;
    volatile uint8_t    en_integral_windup;
    volatile float32_t  velo_setpoint;      /* LAB2 velocity cmd [rad/s]            */
    /* LAB3 trajectory */
    volatile float32_t  pos_setpoint_deg;
    volatile uint8_t    traj_select;        /* 0=Trapezoid  1=S-curve               */
    volatile float32_t  trap_vmax,  trap_amax;
    volatile float32_t  scurve_vmax, scurve_amax, scurve_jmax;
    /* Friction FF */
    volatile float32_t  friction_static;
    volatile float32_t  friction_dynamic;
    volatile uint8_t    friction_en;
    volatile float32_t  disturbance_scale;  /* runtime dist_FF multiplier (1.0 = full, 0.0 = off) */
    /* LAB5 auto-sweep */
    volatile uint8_t    lab5_active;        /* write 1 to start sweep, reads 0 when done          */
    volatile uint8_t    lab5_all_done;      /* goes 1 when all 72×5 moves complete                */
} LAB_Params;

/* =========================================================================
 * API
 * ======================================================================= */

/**
 * @brief  Initialise all control objects (Kalman, cascade, friction, trajectory).
 *         Call once from main(), after all MX_* peripheral inits.
 */
void LAB_Init(void);

/**
 * @brief  Background loop body — handle reset, homing, and P2P trigger.
 *         Call repeatedly inside main() while(1).
 */
void LAB_MainLoop(void);

/**
 * @brief  2 kHz control-loop ISR — registered with TIM20 period-elapsed callback.
 */
void Robot_Period_Control_Loop(TIM_HandleTypeDef *htim);

#endif /* LAB_MANAGER_H */
