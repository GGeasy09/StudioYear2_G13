#ifndef LAB_CONFIG_H
#define LAB_CONFIG_H

#include "trajectory.h"   /* TrajProfile enum for TRAJ_TEST_PROFILE */

/* ============================================================
 *  LAB4 Configuration Header
 *  Edit this file and recompile to change experiment parameters.
 * ============================================================ */

/* ---- LAB SELECTION ----------------------------------------
 *  1 = Kalman Filter test  (open-loop voltage, dual Kalman)
 *  2 = Inner Loop PID      (velocity setpoint, no FF)
 *  3 = Cascade Control     (full trajectory, hard-stop at end)
 *  4 = Dead Band ramp      (0→9V at 0.5V/s, trigger: p2p.trigger=1)
 * ----------------------------------------------------------- */
#define CFG_LAB_SELECT          3

/* ---- KALMAN NOISE ----------------------------------------- */
#define CFG_Q_PROCESS           5.33e-11f
#define CFG_Q_DISTURBANCE       5.00e-12f
#define CFG_R_MEASUREMENT       3e-8f

/* ---- LAB1: Open-loop voltage [V] -------------------------- */
#define CFG_DIRECT_VOLTAGE      0.0f

/* ---- PID GAINS -------------------------------------------- */
#define CFG_KP_IN               10.0f
#define CFG_KI_IN               25.0f
#define CFG_KD_IN               0.0f

#define CFG_KP_OUT              3.0f
#define CFG_KI_OUT              1.0f
#define CFG_KD_OUT              0.0f

/* ---- FEEDFORWARD TAUS ------------------------------------- */
#define CFG_TAU_DIST            0.01f
#define CFG_TAU_REF             0.01f

/* ---- ENABLES (1=on, 0=off) -------------------------------- */
#define CFG_EN_PID              1
#define CFG_EN_DISTURBANCE_FF   1
#define CFG_EN_REFERENCE_FF     1
#define CFG_EN_INTEGRAL_WINDUP  1
#define CFG_EN_FRICTION         1

/* ---- LAB2: Velocity setpoint [rad/s] ---------------------- */
#define CFG_VELO_SETPOINT       0.0f

/* ---- LAB3: Trajectory ------------------------------------- */
#define CFG_POS_SETPOINT_DEG    360.0f
#define CFG_TRAJ_SELECT         1        /* 0=Trapezoid  1=S-curve  2=DeadBand ramp */

#define CFG_TRAP_VMAX           4.05f
#define CFG_TRAP_AMAX           4.8f

#define CFG_SCURVE_VMAX         4.05f
#define CFG_SCURVE_AMAX         4.8f
#define CFG_SCURVE_JMAX         2.0f

/* ---- FRICTION FF ------------------------------------------ */
#define CFG_FRICTION_STATIC     0.71f
#define CFG_FRICTION_DYNAMIC    0.50f

/* ---- DISTURBANCE SCALE ------------------------------------ */
#define CFG_DISTURBANCE_SCALE   1.0f

/* ---- LAB3: Default p2p trajectory parameters -------------- */
#define TRAJ_TEST_PROFILE     TRAJ_PROFILE_SCURVE
#define TRAJ_TEST_TARGET_DEG  360.0f   /* target position    [deg]  */
#define TRAJ_TEST_VMAX        4.05f    /* max velocity       [rad/s]  */
#define TRAJ_TEST_AMAX        4.8f     /* max acceleration   [rad/s²] */
#define TRAJ_TEST_JMAX        3.5f     /* max jerk           [rad/s³] */
#define TRAJ_TEST_TIME        1.5f     /* total duration     [s]      */
#define TRAJ_TEST_ACCT        0.3f     /* accel time (trap)  [s]      */

#endif /* LAB_CONFIG_H */
