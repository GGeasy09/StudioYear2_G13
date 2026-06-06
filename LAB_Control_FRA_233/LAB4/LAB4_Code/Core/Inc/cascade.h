#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include <stdint.h>
#include "arm_math.h"    /* CMSIS-DSP Library */
#include "motor_params.h" /* MOTOR_PARAMS struct */
#include "kalman.h"       /* KALMAN_Multi_Model_Params */
#include "trajectory.h"


typedef struct
{
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t dt;
    float32_t out_min;
    float32_t out_max;
    float32_t integral;
    float32_t prev_error;
    float32_t out;
    float32_t setpoint;
    float32_t measure;
    float32_t error;
} PID;

typedef struct
{
    float32_t Ad1_be;       
    float32_t Bd1_be[2];    
    float32_t Ar2_be[2];    
    float32_t Br2_be[3];    
    float32_t dist_y_prev;  
    float32_t dist_u_prev;  
    float32_t ref_y_prev[2]; 
    float32_t ref_u_prev[2]; 
    float32_t reference_value;
    float32_t disturbance_value;
} Feedforward;

typedef struct
{
    float32_t static_ff;     /* static (breakaway) friction comp  [V]      */
    float32_t dynamic_ff;    /* dynamic (kinetic) friction comp   [V]      */
    float32_t velo_thresh;   /* rad/s — tanh roll-off width for motion dir */
    float32_t pos_thresh;    /* rad   — taper static kick to 0 near target */
    float32_t output;        /* last computed friction FF voltage [V]      */
    uint8_t   moving;        /* 1 = dynamic (moving) regime, 0 = at rest   */
} FrictionFF;

typedef struct
{
    PID         *outer;
    PID         *inner;
    Feedforward *feedforward;
    KALMAN_Multi_Model_Params *Kalman;
    float32_t    pos_setpoint;
    float32_t    velo_setpoint;
    float32_t    pos_state;
    float32_t    velocity_state;
    float32_t    current_state;
    float32_t    disturbance_state;
    int16_t      loop_counter;
    float32_t    pos_error;
    float32_t    velo_error;
    float32_t    voltage_output;
    float32_t	 actual_pos;
    uint8_t      traj_running;   /* 1 = trajectory active — suppresses disturbance decay */
} Cascade;

void CASCADE_Ref_Compute(Cascade* Cascade, float32_t velo_setpoint);
void CASCADE_Disturbance_Compute(Cascade* Cascade, float32_t disturbance);
void CASCADE_Feedforward_Init(MOTOR_PARAMS* motor, Feedforward* ff);
void CASCADE_Controller_Compute_Velocity(PID *pid, float32_t setpoint, float32_t measured);
void CASCADE_Controller_Compute_Position(PID *pid, float32_t setpoint, float32_t measured);
void CASCADE_Controller_Init(float32_t kp, float32_t ki, float32_t kd, PID* controller, float32_t out_min, float32_t out_max, float32_t loop);
void CASCADE_Cascade_Start(Cascade* csc,PID* inner, PID* outer, Feedforward* ff, MOTOR_PARAMS* motor, KALMAN_Multi_Model_Params *kalman);
/* Load the per-tick state estimates / measurement into the cascade struct.
 * Call once each tick before CASCADE_Compute. */
void CASCADE_Update(Cascade *csc, float32_t pos_state, float32_t velocity_state,
                    float32_t current_state, float32_t disturbance_state, float32_t actual_pos);
void CASCADE_Compute(Cascade *csc, float32_t pos_setpoint, float32_t velo_setpoint_override);

/* Friction feedforward — signed, tanh-smoothed. Init once with the comp levels
 * and thresholds; call Compute each tick with velocity and position error.
 * Returns the signed friction-comp voltage (also stored in f->output). */
void      CASCADE_Friction_Init(FrictionFF *f, float32_t static_ff, float32_t dynamic_ff,
                                float32_t velo_thresh, float32_t pos_thresh);
float32_t CASCADE_Friction_Compute(FrictionFF *f, float32_t velocity, float32_t pos_error);
//inner loop is run with 2000Hz outter loop is run with 200Hz I will activate Cascade compute in main by myself with interupth clock 2000Hz by myself
//so make outter loop run every inner loop run 10 time
//Disturbance and reference feedforward run with inner loop use tustin method
//outter loop limit speed is 4.18879 rad/sec and inner loop limit is 12V
#endif // CASCADE_PID_H
