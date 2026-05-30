#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include <math.h>
#include "tim.h"

/* Standard float32_t definition if not using CMSIS */
typedef float float32_t;

typedef enum{
    Trappezoidal = 1,
    SCURVE = 2,
    SCURVE_MIN_JERK = 3
} Trajectory_Profile;

/* Standard output structure for the feedforward and PID loops */
typedef struct {
    float32_t pos;
    float32_t velo;
    float32_t accel;
    int Complete;
    float32_t first_tick;
    TIM_HandleTypeDef *cloak;
} Traj_State_t;

void TRAJ_State_Init(Traj_State_t *traj, TIM_HandleTypeDef *Timer);

/* =========================================================================
 * 1. TRAPEZOIDAL TRAJECTORY (Time-Based)
 * ========================================================================= */
typedef struct {
    float32_t start_pos;
    float32_t setpoint_pos;
    float32_t overall_time;
    float32_t accel_time;
    
    /* Internal Pre-computations */
    float32_t sign;
    float32_t v_max;
    float32_t a_max;
} Traj_Trapezoidal_t;

void TRAJ_Trapezoidal_Plan(Traj_Trapezoidal_t *traj, float32_t start, float32_t setpoint, float32_t t_overall, float32_t t_accel);
Traj_State_t TRAJ_Trapezoidal_Compute(const Traj_Trapezoidal_t *traj, float32_t t);


/* =========================================================================
 * 2.1 S-CURVE (Kinematic Constraint-Based: Velo, Accel, Jerk Limits)
 * Assumes start and end from rest (v=0, a=0)
 * ========================================================================= */
typedef struct {
    float32_t start_pos;
    float32_t setpoint_pos;
    
    /* Internal Pre-computed Segment Times */
    float32_t sign;
    float32_t t_jerk;   /* Time spent ramping accel up/down */
    float32_t t_accel;  /* Time spent at flat max accel */
    float32_t t_velo;   /* Time spent at flat max velocity */
    float32_t t_overall;
    
    float32_t j_max;
    float32_t a_max;
    float32_t v_max;
} Traj_SCurveLimits_t;

void TRAJ_SCurveLimits_Plan(Traj_SCurveLimits_t *traj, float32_t start, float32_t setpoint, float32_t max_v, float32_t max_a, float32_t max_j);
Traj_State_t TRAJ_SCurveLimits_Compute(const Traj_SCurveLimits_t *traj, float32_t t);


/* =========================================================================
 * 2.2 S-CURVE (Minimum Jerk Polynomial: Time-Based)
 * ========================================================================= */
typedef struct {
    float32_t start_pos;
    float32_t setpoint_pos;
    float32_t overall_time;
    float32_t delta_q;
} Traj_MinJerk_t;

void TRAJ_MinJerk_Plan(Traj_MinJerk_t *traj, float32_t start, float32_t setpoint, float32_t t_overall);
Traj_State_t TRAJ_MinJerk_Compute(const Traj_MinJerk_t *traj, float32_t t);

/* =========================================================================
 * 2.3 MINIMUM JERK WITH VELOCITY LIMIT (Auto-computes T from v_max)
 *     Peak velocity of min-jerk polynomial = 1.875 * |delta| / T
 *     So T = 1.875 * |delta| / v_max
 * ========================================================================= */
typedef struct {
    float32_t start_pos;
    float32_t setpoint_pos;
    float32_t overall_time;   /* Auto-computed from v_max */
    float32_t delta_q;
    float32_t v_max;          /* User-specified max velocity (rad/s) */
} Traj_MinJerk_Vlim_t;

void TRAJ_MinJerk_Vlim_Plan(Traj_MinJerk_Vlim_t *traj, float32_t start, float32_t setpoint, float32_t v_max);
Traj_State_t TRAJ_MinJerk_Vlim_Compute(const Traj_MinJerk_Vlim_t *traj, float32_t t);

#endif /* TRAJECTORY_H */