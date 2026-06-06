#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include "arm_math.h"

/* =========================================================================
 * MOTOR_PARAMS — physical motor constants shared by Kalman and Cascade
 * ========================================================================= */
typedef struct {
    float32_t kt;   /* Torque constant       [Nm/A]     */
    float32_t km;   /* Back-EMF constant     [V·s/rad]  */
    float32_t J;    /* Rotor inertia         [kg·m²]    */
    float32_t B;    /* Viscous damping       [Nm·s/rad] */
    float32_t R;    /* Winding resistance    [Ω]        */
    float32_t L;    /* Winding inductance    [H]        */
    float32_t tau;      /* legacy FF filter time const (fallback) [s] */
    float32_t tau_ref;  /* Reference FF LPF time const  [s] — small/fast (clean trajectory input) */
    float32_t tau_dist; /* Disturbance FF LPF time const [s] — large/slow (noisy estimate, in-loop) */
} MOTOR_PARAMS;

#endif /* MOTOR_PARAMS_H */
