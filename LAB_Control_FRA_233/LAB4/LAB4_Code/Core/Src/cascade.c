#include "cascade.h"
#include <math.h>

/* Voltage rail limit (V) */
#define VOLTAGE_MAX    12.0f

/* Outer loop decimation ratio */
#define OUTER_DECIMATE 4
#define BACKLASH_DEADBAND 0.05f
#define TS 0.0005f
#define VELO_INT_DEADBAND 0.05f   /* rad/s — while holding (|setpt| & |err| below this), bleed inner integral to kill stick-slip */

/* -------------------------------------------------------------------------
 * Internal helper
 * ---------------------------------------------------------------------- */
static inline float32_t clampf(float32_t x, float32_t lo, float32_t hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

void CASCADE_Feedforward_Init(MOTOR_PARAMS *motor, Feedforward *ff)
{
    const float32_t kt  = motor->kt;
    const float32_t km  = motor->km;
    const float32_t J   = motor->J;
    const float32_t B   = motor->B;
    const float32_t R   = motor->R;
    const float32_t L   = motor->L;
    const float32_t tau_d = motor->tau_dist;  /* disturbance FF LPF (slow -> noise/loop stability) */
    const float32_t tau_r = motor->tau_ref;   /* reference  FF LPF (fast -> low lag, clean input)  */
    const float32_t Ts  = TS;

    /* ------------------------------------------------------------------ */
    /* Disturbance FF  — uses tau_d (large / slow)                         */
    /* ------------------------------------------------------------------ */
    float32_t D1       = kt * (tau_d + Ts);
    ff->Ad1_be         = (kt * tau_d)  / D1;   /* y[k-1] */
    ff->Bd1_be[0]      = (L + R * Ts)  / D1;   /* u[k]   */
    ff->Bd1_be[1]      = -L            / D1;   /* u[k-1] */

    /* ------------------------------------------------------------------ */
    /* Reference FF  — uses tau_r (small / fast)                           */
    /* ------------------------------------------------------------------ */
    float32_t tauTs    = tau_r + Ts;
    float32_t D2       = km * tauTs * tauTs;

    float32_t JL       = J * L;
    float32_t RJBL     = R * J + B * L;
    float32_t RBktKm   = R * B + kt * km;

    ff->Ar2_be[0]      =  2.0f * km * tau_r * tauTs           / D2; /* y[k-1] */
    ff->Ar2_be[1]      = -(km * tau_r * tau_r)                / D2; /* y[k-2] */

    ff->Br2_be[0]      = (JL + RJBL * Ts + RBktKm * Ts * Ts)  / D2; /* u[k]   */
    ff->Br2_be[1]      = (-2.0f * JL - RJBL * Ts)             / D2; /* u[k-1] */
    ff->Br2_be[2]      =  JL                                   / D2; /* u[k-2] */

    /* ------------------------------------------------------------------ */
    /* Zero all delay registers                                            */
    /* ------------------------------------------------------------------ */
    ff->dist_y_prev    = 0.0f;
    ff->dist_u_prev    = 0.0f;

    ff->ref_y_prev[0]  = 0.0f;
    ff->ref_y_prev[1]  = 0.0f;
    ff->ref_u_prev[0]  = 0.0f;
    ff->ref_u_prev[1]  = 0.0f;

    ff->reference_value   = 0.0f;
    ff->disturbance_value = 0.0f;
}

/* =========================================================================
 * CASCADE_Ref_Compute
 *
 * Reference FF — 2nd-order backward-Euler difference equation.
 * Input  u = csc->velo_setpoint  (rad/s, desired velocity)
 * Output ff->reference_value     (V, voltage feedforward)
 *
 * y[k] = Ar2_be[0]*y[k-1] + Ar2_be[1]*y[k-2]
 *       + Br2_be[0]*u[k]   + Br2_be[1]*u[k-1] + Br2_be[2]*u[k-2]
 * ======================================================================= */
void CASCADE_Ref_Compute(Cascade *csc, float32_t velo_setpoint)
{
    Feedforward *ff = csc->feedforward;
    float32_t    u  = velo_setpoint;

    float32_t y = ff->Ar2_be[0] * ff->ref_y_prev[0]
                + ff->Ar2_be[1] * ff->ref_y_prev[1]
                + ff->Br2_be[0] * u
                + ff->Br2_be[1] * ff->ref_u_prev[0]
                + ff->Br2_be[2] * ff->ref_u_prev[1];

    /* Shift delay registers (newest at index 0) */
    ff->ref_y_prev[1] = ff->ref_y_prev[0];
    ff->ref_y_prev[0] = y;
    ff->ref_u_prev[1] = ff->ref_u_prev[0];
    ff->ref_u_prev[0] = u;

    ff->reference_value = y;
}

/* =========================================================================
 * CASCADE_Disturbance_Compute
 *
 * Disturbance FF — 1st-order backward-Euler difference equation.
 * Input  u = csc->disturbance_state  (estimated load / velocity disturbance)
 * Output ff->disturbance_value       (V, voltage feedforward)
 *
 * y[k] = Ad1_be*y[k-1] + Bd1_be[0]*u[k] + Bd1_be[1]*u[k-1]
 * ======================================================================= */
void CASCADE_Disturbance_Compute(Cascade *csc, float32_t disturbance)
{
    Feedforward *ff = csc->feedforward;
    float32_t    u  = disturbance;

    float32_t y = ff->Ad1_be    * ff->dist_y_prev
                + ff->Bd1_be[0] * u
                + ff->Bd1_be[1] * ff->dist_u_prev;

    ff->dist_y_prev = y;
    ff->dist_u_prev = u;

    ff->disturbance_value = y;
}



/* =========================================================================
 * CASCADE_Controller_Init
 * ======================================================================= */
void CASCADE_Controller_Init(float32_t kp, float32_t ki, float32_t kd,
                              PID       *controller,
                              float32_t  out_min, float32_t out_max,
                              float32_t  loop_period)
{
    controller->kp         = kp;
    controller->ki         = ki;
    controller->kd         = kd;
    controller->dt         = loop_period;
    controller->out_min    = out_min;
    controller->out_max    = out_max;
    controller->integral   = 0.0f;
    controller->prev_error = 0.0f;
    controller->out        = 0.0f;
}

/* =========================================================================
 * CASCADE_Cascade_Start
 * ======================================================================= */
void CASCADE_Cascade_Start(Cascade* csc,PID* inner, PID* outer, Feedforward* ff, MOTOR_PARAMS* motor, KALMAN_Multi_Model_Params *kalman)
{
    csc->inner       = inner;
    csc->outer       = outer;
    csc->feedforward = ff;
    csc->Kalman      = kalman;
    csc->loop_counter = OUTER_DECIMATE;   /* force outer loop on first tick */

    CASCADE_Feedforward_Init(motor, ff);

    csc->pos_setpoint      = 0.0f;
    csc->velo_setpoint     = 0.0f;
    csc->pos_state         = 0.0f;
    csc->velocity_state    = 0.0f;
    csc->current_state     = 0.0f;
    csc->disturbance_state = 0.0f;
    csc->pos_error         = 0.0f;
    csc->velo_error        = 0.0f;
    csc->voltage_output    = 0.0f;
}

/* =========================================================================
 * CASCADE_Update — load this tick's state estimates / measurement into struct
 * ======================================================================= */
void CASCADE_Update(Cascade *csc, float32_t pos_state, float32_t velocity_state,
                    float32_t current_state, float32_t disturbance_state, float32_t actual_pos)
{
    csc->pos_state         = pos_state;
    csc->velocity_state    = velocity_state;
    csc->current_state     = current_state;
    csc->disturbance_state = disturbance_state;
    csc->actual_pos        = actual_pos;
}

/* =========================================================================
 * CASCADE_Compute  — call at inner-loop rate (2 kHz / 0.5 ms)
 * ======================================================================= */
void CASCADE_Compute(Cascade   *csc,
                     float32_t  pos_setpoint,
                     float32_t  velo_setpoint_override)
{
    csc->disturbance_state = csc->Kalman->X[2];
    csc->pos_setpoint = pos_setpoint;
    csc->velo_setpoint = velo_setpoint_override;

    /* ---- Outer position loop (every OUTER_DECIMATE inner ticks) ---- */
    csc->loop_counter++;
    if (csc->loop_counter >= OUTER_DECIMATE)
    {
        csc->loop_counter = 0;
        csc->pos_error    = csc->pos_setpoint - csc->pos_state;
        CASCADE_Controller_Compute_Position(csc->outer, csc->pos_setpoint, csc->pos_state);
    }

    /* ---- Inner velocity loop (every tick) ---- */
    float32_t velo_actual_setpoint = velo_setpoint_override + csc->outer->out;
    CASCADE_Controller_Compute_Velocity(csc->inner, velo_actual_setpoint, csc->velocity_state);

    /* ---- Feedforward ---- */
    CASCADE_Ref_Compute(csc, velo_setpoint_override);
    CASCADE_Disturbance_Compute(csc, csc->Kalman->X[2]);

    /* ---- Clamp disturbance FF to ±5 V before summing ---- */
    csc->feedforward->disturbance_value = clampf(csc->feedforward->disturbance_value, -5.0f, 5.0f);

    /* ---- Sum and clamp ----
     * Disturbance FF is DISABLED: feeding X[2] back as voltage formed a saturating
     * observer loop that ran off to the ±5 V rail and limit-cycled. Kalman stays a
     * pure estimator (X[2] still absorbs model error to keep X[0]/X[1] clean), but
     * it no longer drives the motor. Re-enable only with a well-damped, low-gain
     * disturbance path if ever needed. */
    csc->voltage_output = csc->inner->out
                        + csc->feedforward->reference_value
                        + csc->feedforward->disturbance_value;  // disabled */




    csc->voltage_output = clampf(csc->voltage_output, -VOLTAGE_MAX, VOLTAGE_MAX);
}

/* =========================================================================
 * CASCADE_Friction_Init — set friction-comp levels and thresholds
 * ======================================================================= */
void CASCADE_Friction_Init(FrictionFF *f, float32_t static_ff, float32_t dynamic_ff,
                           float32_t velo_thresh, float32_t pos_thresh)
{
    f->static_ff   = static_ff;
    f->dynamic_ff  = dynamic_ff;
    f->velo_thresh = velo_thresh;
    f->pos_thresh  = pos_thresh;
    f->output      = 0.0f;
    f->moving      = 0;
}

/* =========================================================================
 * CASCADE_Friction_Compute — signed, tanh-smoothed friction feedforward
 *
 *   Moving  : friction opposes motion -> push in direction of velocity.
 *   At rest : push in direction of position error to break stiction.
 *   tanh() gives a smooth +/-1 sign that fades to 0 near zero, so the kick
 *   self-tapers as the shaft settles on target.
 *   Returns the signed comp voltage (also stored in f->output).
 * ======================================================================= */
float32_t CASCADE_Friction_Compute(FrictionFF *f, float32_t velocity, float32_t pos_error)
{
    /* These return -1.0 to +1.0. The sign indicates direction automatically! */
    float32_t dir_move = tanhf(velocity  / f->velo_thresh);
    float32_t dir_rest = tanhf(pos_error / f->pos_thresh);

    /* move_w is strictly 0.0 to 1.0. It acts purely as a percentage of "how fast are we moving?" */
    float32_t move_w = fabsf(dir_move);

    /* Calculate friction magnitude. Blends between static and dynamic. */
    float32_t fc_mag = f->dynamic_ff
                     + (f->static_ff - f->dynamic_ff) * (1.0f - move_w);

    float32_t activation = dir_move + (1.0f - move_w) * dir_rest;

    /* Final output carries the correct sign and magnitude */
    f->output = fc_mag * activation;
    f->moving = (move_w > 0.5f);

    return f->output;
}

/* =========================================================================
 * CASCADE_Controller_Compute  — PID one step
 * ======================================================================= */
void CASCADE_Controller_Compute_Position(PID *pid, float32_t setpoint, float32_t measured)
{
    float32_t internal_ki = pid->ki;
    float32_t error  = setpoint - measured;
    float32_t p_term = pid->kp * error;
    // if(fabsf(error) < 0.5 && fabsf(error) >= 0.0017){
    //     internal_ki = 1.0f;
    // }
    float32_t i_term = pid->integral + internal_ki * error * pid->dt;
    float32_t d_term = pid->kd * (error - pid->prev_error) / pid->dt;

    pid->setpoint = setpoint;
    pid->measure  = measured;
    pid->error    = error;

    float32_t out = p_term + i_term + d_term;

    // /* Near-zero dead-zone: softly decay integral, do NOT corrupt prev_error */
    int integral_frozen = 0;
    if (fabsf(error) < 0.0017f)
    {
        pid->integral  *= 0.75f;
        integral_frozen = 1;
    }

    /* Anti-windup: conditional integration */
    float32_t out_clamped = clampf(out, pid->out_min, pid->out_max);
    if ((out == out_clamped) && !integral_frozen)
    {
        pid->integral = i_term;
    }

    pid->out        = out_clamped;
    pid->prev_error = error;
}

void CASCADE_Controller_Compute_Velocity(PID *pid, float32_t setpoint, float32_t measured)
{

    float32_t error  = setpoint - measured;
    float32_t p_term = pid->kp * error;

    float32_t i_term = pid->integral + pid->ki * error * pid->dt;
    float32_t d_term = pid->kd * (error - pid->prev_error) / pid->dt;

    pid->setpoint = setpoint;
    pid->measure  = measured;
    pid->error    = error;

    float32_t out = p_term + i_term + d_term;

    /* Integral deadband: only while HOLDING (commanded velocity ~0 AND error tiny),
     * bleed the integrator instead of accumulating. Stops it winding up against
     * static friction and triggering stick-slip lurches at the setpoint. The
     * setpoint gate keeps this inactive during trajectory moves. */
    int integral_frozen = 0;
//    if (fabsf(setpoint) < VELO_INT_DEADBAND && fabsf(error) < VELO_INT_DEADBAND)
//    {
//        pid->integral *= 0.9f;
//        integral_frozen = 1;
//    }

    /* Anti-windup: conditional integration */
    float32_t out_clamped = clampf(out, pid->out_min, pid->out_max);

  if (out == out_clamped && !integral_frozen)
    {
        pid->integral = i_term;
    }
    pid->out        = out_clamped;
    pid->prev_error = error;
}
