#include "trajectory.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Helper to prevent dangerous divisions by zero */
static inline float32_t safe_time(float32_t t) {
    return (t < 0.0001f) ? 0.0001f : t;
}

/* ---- Small math helpers used by the trajectory manager ---- */
static inline float32_t deg2rad(float32_t deg) { return deg * (M_PI / 180.0f); }
static inline float32_t rad2deg(float32_t rad) { return rad * (180.0f / M_PI); }
static inline float32_t clampf_t(float32_t x, float32_t lo, float32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* =========================================================================
 * 1. TRAPEZOIDAL IMPLEMENTATION
 * ========================================================================= */

void TRAJ_State_Init(Traj_State_t *traj, TIM_HandleTypeDef *Timer){
    traj->cloak = Timer;
}

void TRAJ_Trapezoidal_Plan(Traj_Trapezoidal_t *traj, float32_t start, float32_t setpoint, float32_t t_overall, float32_t t_accel) 
{
    traj->start_pos = start;
    traj->setpoint_pos = setpoint;
    traj->overall_time = safe_time(t_overall);
    
    /* Geometry constraint: Accel time cannot exceed half of total time */
    float32_t max_t_accel = traj->overall_time * 0.5f;
    traj->accel_time = (t_accel > max_t_accel) ? max_t_accel : safe_time(t_accel);
    
    float32_t delta = setpoint - start;
    traj->sign = (delta >= 0.0f) ? 1.0f : -1.0f;
    delta = fabsf(delta);
    
    /* Peak velocity required to cover distance under trapezoid area */
    traj->v_max = delta / (traj->overall_time - traj->accel_time);
    traj->a_max = traj->v_max / traj->accel_time;
}

Traj_State_t TRAJ_Trapezoidal_Compute(const Traj_Trapezoidal_t *traj, float32_t t) 
{
    Traj_State_t out;
    float32_t t_total = traj->overall_time;
    float32_t t_accel = traj->accel_time;
    float32_t t_flat  = t_total - t_accel;
    
    /* Clamp time to end of move */
    if (t >= t_total) {
        out.pos = traj->setpoint_pos;
        out.velo = 0.0f;
        out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }
    
    if (t <= 0.0f) {
        out.pos = traj->start_pos;
        out.velo = 0.0f;
        out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }

    float32_t pos_abs = 0.0f;
    float32_t vel_abs = 0.0f;
    float32_t acc_abs = 0.0f;

    /* Phase 1: Acceleration */
    if (t < t_accel) {
        acc_abs = traj->a_max;
        vel_abs = acc_abs * t;
        pos_abs = 0.5f * acc_abs * t * t;
    }
    /* Phase 2: Constant Velocity */
    else if (t < t_flat) {
        acc_abs = 0.0f;
        vel_abs = traj->v_max;
        float32_t pos_phase1_end = 0.5f * traj->a_max * t_accel * t_accel;
        pos_abs = pos_phase1_end + vel_abs * (t - t_accel);
    }
    /* Phase 3: Deceleration */
    else {
        float32_t t_dec = t - t_flat;
        acc_abs = -traj->a_max;
        vel_abs = traj->v_max + (acc_abs * t_dec);
        
        float32_t pos_phase1_end = 0.5f * traj->a_max * t_accel * t_accel;
        float32_t pos_phase2_end = pos_phase1_end + traj->v_max * (t_flat - t_accel);
        pos_abs = pos_phase2_end + (traj->v_max * t_dec) + (0.5f * acc_abs * t_dec * t_dec);
    }

    /* Apply directional sign */
    out.pos   = traj->start_pos + (traj->sign * pos_abs);
    out.velo  = traj->sign * vel_abs;
    out.accel = traj->sign * acc_abs;
    out.Complete = 0;
    return out;
}


/* =========================================================================
 * 2.1 S-CURVE LIMITS (7-Segment Analytic Solver from Rest)
 * ========================================================================= */

void TRAJ_SCurveLimits_Plan(Traj_SCurveLimits_t *traj, float32_t start, float32_t setpoint, float32_t max_v, float32_t max_a, float32_t max_j) 
{
    traj->start_pos = start;
    traj->setpoint_pos = setpoint;
    
    float32_t delta = setpoint - start;
    traj->sign = (delta >= 0.0f) ? 1.0f : -1.0f;
    delta = fabsf(delta);
    
    traj->j_max = fabsf(max_j);
    float32_t a_max_local = fabsf(max_a);
    float32_t v_max_local = fabsf(max_v);

    if (delta < 0.00001f) {
        traj->t_jerk = 0; traj->t_accel = 0; traj->t_velo = 0; traj->t_overall = 0;
        return;
    }

    /* Pass 1: Check if max acceleration can be reached given max velocity */
    if ((v_max_local * traj->j_max) < (a_max_local * a_max_local)) {
        a_max_local = sqrtf(v_max_local * traj->j_max);
    }

    /* Time to ramp up acceleration */
    float32_t t_j = a_max_local / traj->j_max;
    /* Time at flat acceleration */
    float32_t t_a = (v_max_local / a_max_local) - t_j;
    
    /* Distance required to reach v_max_local (accel + decel phases).
     * Each side covers v_max*(2*t_j + t_a)/2, so both sides = v_max*(2*t_j + t_a). */
    float32_t dist_accel = v_max_local * (2.0f * t_j + t_a);

    /* Pass 2: Check if max velocity can be reached given the overall distance */
    if (delta < dist_accel) {
        /* Move is too short to reach peak velocity; reduce v_max and re-evaluate */
        v_max_local = (-a_max_local * a_max_local + sqrtf((a_max_local * a_max_local * a_max_local * a_max_local) + 4.0f * traj->j_max * traj->j_max * a_max_local * delta)) / (2.0f * traj->j_max);
        
        /* Check if max acceleration can still be reached */
        if ((v_max_local * traj->j_max) < (a_max_local * a_max_local)) {
            a_max_local = cbrtf(delta * traj->j_max * traj->j_max * 0.5f);
            t_j = a_max_local / traj->j_max;
            t_a = 0.0f;
        } else {
            t_j = a_max_local / traj->j_max;
            t_a = (v_max_local / a_max_local) - t_j;
        }
        traj->t_velo = 0.0f;
    } else {
        /* Max velocity is reached; calculate time spent at flat max velocity */
        traj->t_velo = (delta - dist_accel) / v_max_local;
    }

    traj->a_max = a_max_local;
    traj->v_max = v_max_local;
    traj->t_jerk  = t_j;
    traj->t_accel = t_a;
    traj->t_overall = (4.0f * t_j) + (2.0f * t_a) + traj->t_velo;
}

Traj_State_t TRAJ_SCurveLimits_Compute(const Traj_SCurveLimits_t *traj, float32_t t) 
{
    Traj_State_t out;
    
    if (t >= traj->t_overall) {
        out.pos = traj->setpoint_pos; out.velo = 0.0f; out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }
    if (t <= 0.0f) {
        out.pos = traj->start_pos; out.velo = 0.0f; out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }

    float32_t tj = traj->t_jerk;
    float32_t ta = traj->t_accel;
    float32_t tv = traj->t_velo;
    float32_t j  = traj->j_max;

    /* Segment thresholds */
    float32_t T1 = tj;
    float32_t T2 = T1 + ta;
    float32_t T3 = T2 + tj;
    float32_t T4 = T3 + tv;
    float32_t T5 = T4 + tj;
    float32_t T6 = T5 + ta;

    float32_t p = 0, v = 0, a = 0;

    /* 1. Accel Ramp Up */
    if (t < T1) {
        a = j * t;
        v = 0.5f * j * t * t;
        p = j * t * t * t / 6.0f;
    }
    /* 2. Constant Accel */
    else if (t < T2) {
        float32_t dt = t - T1;
        a = traj->a_max;
        v = (0.5f * j * tj * tj) + (a * dt);
        p = (j * tj * tj * tj / 6.0f) + (0.5f * j * tj * tj * dt) + (0.5f * a * dt * dt);
    }
    /* 3. Accel Ramp Down */
    else if (t < T3) {
        float32_t dt = t - T2;
        float32_t v2 = (0.5f * j * tj * tj) + (traj->a_max * ta);
        float32_t p2 = (j * tj * tj * tj / 6.0f) + (0.5f * j * tj * tj * ta) + (0.5f * traj->a_max * ta * ta);
        a = traj->a_max - (j * dt);
        v = v2 + (traj->a_max * dt) - (0.5f * j * dt * dt);
        p = p2 + (v2 * dt) + (0.5f * traj->a_max * dt * dt) - (j * dt * dt * dt / 6.0f);
    }
    /* 4. Constant Velocity */
    else if (t < T4) {
        float32_t dt = t - T3;
        float32_t dist_accel = traj->v_max * (2.0f * tj + ta) * 0.5f; // accel-phase distance
        a = 0.0f;
        v = traj->v_max;
        p = dist_accel + (v * dt);
    }
    /* 5-7: Symmetric Deceleration uses geometry reversed from target */
    else {
        float32_t t_rem = traj->t_overall - t;
        float32_t p_rev = 0, v_rev = 0, a_rev = 0;
        
        /* Evaluate mirrored acceleration from the end point backward */
        if (t_rem < T1) {
            a_rev = j * t_rem;
            v_rev = 0.5f * j * t_rem * t_rem;
            p_rev = j * t_rem * t_rem * t_rem / 6.0f;
        } else if (t_rem < T2) {
            float32_t dt = t_rem - T1;
            a_rev = traj->a_max;
            v_rev = (0.5f * j * tj * tj) + (a_rev * dt);
            p_rev = (j * tj * tj * tj / 6.0f) + (0.5f * j * tj * tj * dt) + (0.5f * a_rev * dt * dt);
        } else {
            float32_t dt = t_rem - T2;
            float32_t v2 = (0.5f * j * tj * tj) + (traj->a_max * ta);
            float32_t p2 = (j * tj * tj * tj / 6.0f) + (0.5f * j * tj * tj * ta) + (0.5f * traj->a_max * ta * ta);
            a_rev = traj->a_max - (j * dt);
            v_rev = v2 + (traj->a_max * dt) - (0.5f * j * dt * dt);
            p_rev = p2 + (v2 * dt) + (0.5f * traj->a_max * dt * dt) - (j * dt * dt * dt / 6.0f);
        }
        
        float32_t total_delta = fabsf(traj->setpoint_pos - traj->start_pos);
        p = total_delta - p_rev;
        v = v_rev;
        a = -a_rev;
    }

    out.pos   = traj->start_pos + (traj->sign * p);
    out.velo  = traj->sign * v;
    out.accel = traj->sign * a;
    out.Complete = 0;
    return out;
}


/* =========================================================================
 * 2.2 MINIMUM JERK IMPLEMENTATION (5th-Order Polynomial)
 * ========================================================================= */

void TRAJ_MinJerk_Plan(Traj_MinJerk_t *traj, float32_t start, float32_t setpoint, float32_t t_overall) 
{
    traj->start_pos = start;
    traj->setpoint_pos = setpoint;
    traj->overall_time = safe_time(t_overall);
    traj->delta_q = setpoint - start;
}

Traj_State_t TRAJ_MinJerk_Compute(const Traj_MinJerk_t *traj, float32_t t) 
{
    Traj_State_t out;
    float32_t T = traj->overall_time;
    
    if (t >= T) {
        out.pos = traj->setpoint_pos; out.velo = 0.0f; out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }
    if (t <= 0.0f) {
        out.pos = traj->start_pos; out.velo = 0.0f; out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }

    /* Normalized time tau inside [0, 1] */
    float32_t tau  = t / T;
    float32_t tau2 = tau * tau;
    float32_t tau3 = tau2 * tau;
    float32_t tau4 = tau3 * tau;
    float32_t tau5 = tau4 * tau;

    /* Pre-calculate common division coefficients */
    float32_t inv_T  = 1.0f / T;
    float32_t inv_T2 = inv_T * inv_T;

    /* 5th order polynomial evaluation */
    out.pos   = traj->start_pos + traj->delta_q * ((10.0f * tau3) - (15.0f * tau4) + (6.0f * tau5));
    out.velo  = traj->delta_q * inv_T  * ((30.0f * tau2) - (60.0f * tau3) + (30.0f * tau4));
    out.accel = traj->delta_q * inv_T2 * ((60.0f * tau)  - (180.0f * tau2) + (120.0f * tau3));
    out.Complete = 0;

    return out;
}

/* =========================================================================
 * 2.3 MINIMUM JERK WITH VELOCITY LIMIT
 *     Peak of 5th-order poly always at tau=0.5: v_peak = 1.875 * delta / T
 *     Solving for T: T = 1.875 * |delta| / v_max
 * ========================================================================= */

void TRAJ_MinJerk_Vlim_Plan(Traj_MinJerk_Vlim_t *traj, float32_t start, float32_t setpoint, float32_t v_max)
{
    traj->start_pos    = start;
    traj->setpoint_pos = setpoint;
    traj->delta_q      = setpoint - start;
    traj->v_max        = fabsf(v_max);

    float32_t delta = fabsf(traj->delta_q);

    /* Degenerate case — already at target */
    if (delta < 0.00001f) {
        traj->overall_time = 0.0001f;
        return;
    }

    /* T = 1.875 * |delta| / v_max */
    traj->overall_time = safe_time(1.875f * delta / traj->v_max);
}

Traj_State_t TRAJ_MinJerk_Vlim_Compute(const Traj_MinJerk_Vlim_t *traj, float32_t t)
{
    Traj_State_t out;
    float32_t T = traj->overall_time;

    if (t >= T) {
        out.pos = traj->setpoint_pos; out.velo = 0.0f; out.accel = 0.0f;
        out.Complete = 1;
        return out;
    }
    if (t <= 0.0f) {
        out.pos = traj->start_pos; out.velo = 0.0f; out.accel = 0.0f;
        out.Complete = 0;
        return out;
    }

    float32_t tau  = t / T;
    float32_t tau2 = tau  * tau;
    float32_t tau3 = tau2 * tau;
    float32_t tau4 = tau3 * tau;
    float32_t tau5 = tau4 * tau;

    float32_t inv_T  = 1.0f / T;
    float32_t inv_T2 = inv_T * inv_T;

    out.pos   = traj->start_pos + traj->delta_q * ((10.0f * tau3) - (15.0f * tau4) + (6.0f * tau5));
    out.velo  = traj->delta_q * inv_T  * ((30.0f * tau2) - (60.0f * tau3) + (30.0f * tau4));
    out.accel = traj->delta_q * inv_T2 * ((60.0f * tau)  - (180.0f * tau2) + (120.0f * tau3));
    out.Complete = 0;

    return out;
}

/* =========================================================================
 * TRAJECTORY MANAGER IMPLEMENTATION
 * ========================================================================= */

/* Dynamic-time tuning for min-jerk moves */
#define TRAJ_DYN_SPEED_DEG_S  110.0f   /* nominal cruise speed (deg/s) */
#define TRAJ_DYN_T_MIN        0.9f     /* clamp: shortest allowed move (s) */
#define TRAJ_DYN_T_MAX        3.5f     /* clamp: longest allowed move (s) */

void TrajManager_Init(TrajManager *mgr, TIM_HandleTypeDef *htim)
{
    TRAJ_State_Init(&mgr->state, htim);
    mgr->state.Complete = 1;
    mgr->profile        = TRAJ_PROFILE_MINJERK;
    mgr->elapsed        = 0.0f;
    mgr->running        = 0;
    mgr->cur_pos_rad    = 0.0f;

    /* Pre-arm all planners with safe zero-move defaults */
    TRAJ_MinJerk_Plan      (&mgr->min_jerk,      0.0f, 0.0f, 1.0f);
    TRAJ_MinJerk_Vlim_Plan (&mgr->min_jerk_vlim, 0.0f, 0.0f, 0.5f);
    TRAJ_Trapezoidal_Plan  (&mgr->trapezoid,     0.0f, 0.0f, 1.0f, 0.3f);
    TRAJ_SCurveLimits_Plan (&mgr->scurve,        0.0f, 0.0f, 0.5f, 2.0f, 10.0f);
}

float32_t TrajManager_DynTime(float32_t target_deg, float32_t current_rad)
{
    float32_t dist = fabsf(target_deg - rad2deg(current_rad));
    return clampf_t(dist / TRAJ_DYN_SPEED_DEG_S, TRAJ_DYN_T_MIN, TRAJ_DYN_T_MAX);
}

void TrajManager_Plan(TrajManager *mgr, int profile,
                      float32_t target_deg,
                      float32_t p1, float32_t p2, float32_t p3,
                      float32_t current_rad)
{
    float32_t start  = current_rad;
    float32_t target = deg2rad(target_deg);

    /* Substitute sane defaults for unspecified parameters */
    if (p1 < 0.001f) p1 = 0.5f;
    if (p2 < 0.001f) p2 = 2.0f;
    if (p3 < 0.001f) p3 = 10.0f;

    mgr->profile = profile;

    switch (profile)
    {
        default:
        case TRAJ_PROFILE_MINJERK:
            TRAJ_MinJerk_Plan(&mgr->min_jerk, start, target, p1);
            break;
        case TRAJ_PROFILE_MINJERK_VLIM:
            TRAJ_MinJerk_Vlim_Plan(&mgr->min_jerk_vlim, start, target, p1);
            break;
        case TRAJ_PROFILE_TRAPEZOID:
            TRAJ_Trapezoidal_Plan(&mgr->trapezoid, start, target, p1, p2);
            break;
        case TRAJ_PROFILE_SCURVE:
            TRAJ_SCurveLimits_Plan(&mgr->scurve, start, target, p1, p2, p3);
            break;
    }

    mgr->elapsed        = 0.0f;
    mgr->state.Complete = 0;
    mgr->running        = 1;
    mgr->cur_pos_rad    = target;
}

Traj_State_t TrajManager_Step(TrajManager *mgr)
{
    mgr->elapsed += 0.0005f;   /* 2 kHz inner loop period */

    Traj_State_t ref;
    switch (mgr->profile)
    {
        default:
        case TRAJ_PROFILE_MINJERK:
            ref = TRAJ_MinJerk_Compute     (&mgr->min_jerk,      mgr->elapsed); break;
        case TRAJ_PROFILE_MINJERK_VLIM:
            ref = TRAJ_MinJerk_Vlim_Compute(&mgr->min_jerk_vlim, mgr->elapsed); break;
        case TRAJ_PROFILE_TRAPEZOID:
            ref = TRAJ_Trapezoidal_Compute (&mgr->trapezoid,     mgr->elapsed); break;
        case TRAJ_PROFILE_SCURVE:
            ref = TRAJ_SCurveLimits_Compute(&mgr->scurve,        mgr->elapsed); break;
    }

    if (ref.Complete)
    {
        mgr->state.Complete = 1;
        mgr->running        = 0;
    }

    mgr->state = ref;
    return ref;
}
