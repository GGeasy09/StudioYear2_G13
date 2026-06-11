#ifndef ROBOT_PROCESS_H
#define ROBOT_PROCESS_H

/* =============================================================================
 * robot_process.h — robot state-machine (non-real-time, runs in the main loop).
 *
 * The state-machine logic was lifted out of main.c. The globals it operates on
 * are still DEFINED in main.c; this header shares them via `extern` so the
 * implementation can live in robot_process.c.
 *
 * To finish the split:
 *   1. #include "robot_process.h" in main.c.
 *   2. In main.c, DELETE the SeqState / TestState / JoyEdges typedefs below
 *      (they now live here) — keep the variable definitions (SeqState seq = {0};).
 *   3. Cut the function bodies (Robot_HandleEmergency ... Robot_State_Process)
 *      out of main.c into robot_process.c, which #includes this header.
 *   4. The tuning #defines below are duplicated from main.c; identical object-like
 *      macros are legal to redefine, but you can delete the main.c copies to be tidy.
 * ============================================================================= */

#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "BaseSystem.h"
#include "elec_cabient.h"
#include "kalman.h"
#include "trajectory.h"
#include "cascade.h"
#include "system_state.h"

/* ===== Shared tuning / geometry #defines (from main.c) ===== */
#define DEBUG_MODE 0
#define DEG_PER_HOLE        5.0f
#define HOLE_COUNT          72
#define GRIPPER_TIMEOUT_MS  5000U
#define PROFILE_MINJERK      0
#define PROFILE_MINJERK_VLIM 1
#define PROFILE_TRAPEZOID    2
#define PROFILE_SCURVE       3
#define P2P_TUNE_PROFILE    PROFILE_SCURVE
#define P2P_TUNE_VMAX       4.05f   /* rad/s   — synced from LAB3 */
#define P2P_TUNE_AMAX       4.8f    /* rad/s²  — synced from LAB3 */
#define P2P_TUNE_JMAX       2.0f
#define P2P_TUNE_TIME       1.5f
#define P2P_TUNE_ACCT       0.3f
#define TRAJ_VMAX  4.05f   /* rad/s   — synced from LAB3 */
#define TRAJ_AMAX  4.8f    /* rad/s²  — synced from LAB3 */
#define TRAJ_JMAX  3.5f

/* ===== State structs (move these OUT of main.c) ===== */
typedef struct {
    int      active;               /* 1 = sequence currently running          */
    int      step;                 /* current pair index (0..Pairs-1)         */
    int      phase;                /* 0..5 pick/place phase                    */
    uint32_t settle_phase_start;   /* tick settle phase began (1.5 s timeout)  */
    uint32_t settle_ok_start;      /* tick error first went below threshold    */
} SeqState;

typedef struct {
    int      active;               /* 1 = test running                        */
    int      repeat;               /* completed round-trips so far             */
    int      phase;                /* PERFORM/PRECISION phase index            */
    uint32_t delay_start;          /* timestamp for inter-move delay           */
} TestState;

typedef struct {
    uint8_t white_left;
    uint8_t white_right;
    uint8_t black;
    uint8_t blue;
    uint8_t yellow;
    uint8_t red;
} JoyEdges;

/* ===== Globals (DEFINED in main.c, shared here) ===== */
extern SYSTEM_STATE               sys_state;
extern KALMAN_Multi_Model_Params  Kalman;
extern Encoder                    encoder;
extern Proximity                  ref_pos;
extern TrajManager                traj_mgr;
extern Joystick                   robot_joy;
extern Pilot_ramp                 robot_ramp;
extern Gripper                    robot_gripper;
extern SeqState                   seq;
extern TestState                  test;
extern JoyEdges                   joy_prev;
extern uint8_t                    emergency_latch;
extern uint8_t                    joy_picking;
extern uint8_t                    joy_placing;
extern uint8_t                    p2p_grip_pending;
extern uint32_t                   color_hold_start;
extern PID                        Inner;
extern PID                        Outer;

/* Note: BaseCmd, registerFrame, htim20, and the command/trust/state enums
 * already come from the included headers (BaseSystem.h, elec_cabient.h,
 * system_state.h, tim.h) -- no extern needed here. If the compiler reports any
 * of them undefined in robot_process.c, add an extern for it. */

/* ===== Entry point — call once per main-loop iteration ===== */
void Robot_State_Process(void);

#endif /* ROBOT_PROCESS_H */
