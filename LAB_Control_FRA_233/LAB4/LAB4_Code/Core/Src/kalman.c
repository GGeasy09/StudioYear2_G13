#include "kalman.h"

#define KALMAN_FREQ  0.0005f
#define INV_SINGULAR_THRESHOLD  1e-10f   // below this we skip the update (avoid divide-by-zero / hang)

/* ============================================================
 *  INIT
 * ============================================================ */
void KALMAN_Multi_Model_Init(KALMAN_Multi_Model_Params* params,
                             float32_t process_noise, float32_t disturbance_noise,
                             float32_t measurement_noise,
                             MOTOR_PARAMS* parameter)
{
    int i;
    params->Process_Noise = process_noise;

    /* --- Store motor reference for acceleration calculation --- */
    params->motor          = parameter;
    params->accel_estimate = 0.0f;

    /* --- H: measurement matrix  [1 x 4]  maps position state to measurement --- */
    float32_t H_init[MEASUREMENT_SIZE * STATE_SIZE] = {
        1.0f, 0.0f, 0.0f, 0.0f
    };
    for (i = 0; i < MEASUREMENT_SIZE * STATE_SIZE; i++) { params->H[i] = H_init[i]; }

    /* --- F: state transition matrix  [4 x 4] --- */
    float32_t F_init[STATE_SIZE * STATE_SIZE] = {
        1.0f,  KALMAN_FREQ, 0.0f, 0.0f,
        0.0f,  1.0f - (parameter->B / parameter->J * KALMAN_FREQ),  -KALMAN_FREQ / parameter->J, parameter->kt / parameter->J * KALMAN_FREQ,
        0.0f,  0.0f, 1.0f, 0.0f,
        0.0f,  -parameter->km / parameter->L * KALMAN_FREQ, 0.0f, 1.0f - parameter->R / parameter->L * KALMAN_FREQ
    };
    for (i = 0; i < STATE_SIZE * STATE_SIZE; i++) { params->F[i] = F_init[i]; }

    /* --- Q: process noise covariance  [4 x 4] --- */
    float32_t Q_init[STATE_SIZE * STATE_SIZE] = {
        0.0f,  0.0f,                                                                              0.0f,             0.0f,
        0.0f,  KALMAN_FREQ * KALMAN_FREQ / parameter->J / parameter->J * process_noise,  -KALMAN_FREQ / parameter->J * process_noise,  0.0f,
        0.0f,  -KALMAN_FREQ / parameter->J * process_noise,                               process_noise,    0.0f,
        0.0f,  0.0f,                                                                              0.0f,             0.0f
    };
    for (i = 0; i < STATE_SIZE * STATE_SIZE; i++) { params->Q[i] = Q_init[i]; }

    /* --- R: measurement noise covariance  [1 x 1] --- */
    params->R[0] = measurement_noise;

    /* --- G: control input matrix  [4 x 1] --- */
    float32_t G_init[STATE_SIZE * INPUT_SIZE] = { 0.0f, 0.0f, 0.0f, KALMAN_FREQ / parameter->L };
    for (i = 0; i < STATE_SIZE * INPUT_SIZE; i++) { params->G[i] = G_init[i]; }

    /* --- Initial state X = 0 --- */
    for (i = 0; i < STATE_SIZE; i++) { params->X[i] = 0.0f; }

    /* --- Initial covariance P = I --- */
    float32_t P_init[STATE_SIZE * STATE_SIZE] = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    for (i = 0; i < STATE_SIZE * STATE_SIZE; i++) { params->P[i] = P_init[i]; }

    /* --- Identity matrix --- */
    float32_t I_init[STATE_SIZE * STATE_SIZE] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    for (i = 0; i < STATE_SIZE * STATE_SIZE; i++) { params->Iden[i] = I_init[i]; }

    /* ---- Init arm_matrix instances ---- */
    arm_mat_init_f32(&params->X_matrix,    STATE_SIZE,       1,                params->X);
    arm_mat_init_f32(&params->P_matrix,    STATE_SIZE,       STATE_SIZE,       params->P);
    arm_mat_init_f32(&params->Q_matrix,    STATE_SIZE,       STATE_SIZE,       params->Q);
    arm_mat_init_f32(&params->F_matrix,    STATE_SIZE,       STATE_SIZE,       params->F);
    arm_mat_init_f32(&params->F_trans_matrix, STATE_SIZE,    STATE_SIZE,       params->F_trans);
    arm_mat_init_f32(&params->H_matrix,    MEASUREMENT_SIZE, STATE_SIZE,       params->H);
    arm_mat_init_f32(&params->H_trans_matrix, STATE_SIZE,    MEASUREMENT_SIZE, params->H_trans);
    arm_mat_init_f32(&params->R_matrix,    MEASUREMENT_SIZE, MEASUREMENT_SIZE, params->R);
    arm_mat_init_f32(&params->K_matrix,    STATE_SIZE,       MEASUREMENT_SIZE, params->K);
    arm_mat_init_f32(&params->K_trans_matrix, MEASUREMENT_SIZE, STATE_SIZE,    params->K_trans);
    arm_mat_init_f32(&params->Z_matrix,    MEASUREMENT_SIZE, 1,                params->Z);
    arm_mat_init_f32(&params->G_matrix,    STATE_SIZE,       INPUT_SIZE,       params->G);
    arm_mat_init_f32(&params->Iden_matrix, STATE_SIZE,       STATE_SIZE,       params->Iden);

    /* Step 1 */
    arm_mat_init_f32(&params->FX_matrix,    STATE_SIZE, 1,          params->FX);
    arm_mat_init_f32(&params->Gu_matrix,    STATE_SIZE, 1,          params->Gu);
    arm_mat_init_f32(&params->X_new_matrix, STATE_SIZE, 1,          params->X_new);

    /* Step 2 */
    arm_mat_init_f32(&params->FP_matrix,        STATE_SIZE, STATE_SIZE, params->FP);
    arm_mat_init_f32(&params->FPF_trans_matrix,  STATE_SIZE, STATE_SIZE, params->FPF_trans);

    /* Step 3 */
    arm_mat_init_f32(&params->HP_matrix,             MEASUREMENT_SIZE, STATE_SIZE,       params->HP);
    arm_mat_init_f32(&params->HPH_trans_matrix,      MEASUREMENT_SIZE, MEASUREMENT_SIZE, params->HPH_trans);
    arm_mat_init_f32(&params->HPH_trans_R_matrix,    MEASUREMENT_SIZE, MEASUREMENT_SIZE, params->HPH_trans_R);
    arm_mat_init_f32(&params->HPH_trans_R_inv_matrix, MEASUREMENT_SIZE, MEASUREMENT_SIZE, params->HPH_trans_R_inv);
    arm_mat_init_f32(&params->PH_trans_matrix,       STATE_SIZE,       MEASUREMENT_SIZE, params->PH_trans);

    /* Step 4 */
    arm_mat_init_f32(&params->HX_matrix,    MEASUREMENT_SIZE, 1, params->HX);
    arm_mat_init_f32(&params->Z_HX_matrix,  MEASUREMENT_SIZE, 1, params->Z_HX);
    arm_mat_init_f32(&params->KZ_HX_matrix, STATE_SIZE,       1, params->KZ_HX);

    /* Step 5 */
    arm_mat_init_f32(&params->KH_matrix,              STATE_SIZE, STATE_SIZE,       params->KH);
    arm_mat_init_f32(&params->IKH_matrix,             STATE_SIZE, STATE_SIZE,       params->IKH);
    arm_mat_init_f32(&params->IKH_trans_matrix,       STATE_SIZE, STATE_SIZE,       params->IKH_trans);
    arm_mat_init_f32(&params->IKHP_matrix,            STATE_SIZE, STATE_SIZE,       params->IKHP);
    arm_mat_init_f32(&params->IKHPIKHP_trans_matrix,  STATE_SIZE, STATE_SIZE,       params->IKHPIKHP_trans);
    arm_mat_init_f32(&params->KR_matrix,              STATE_SIZE, MEASUREMENT_SIZE, params->KR);
    arm_mat_init_f32(&params->KRK_trans_matrix,       STATE_SIZE, STATE_SIZE,       params->KRK_trans);
    arm_mat_init_f32(&params->KH_trans_matrix,        STATE_SIZE, STATE_SIZE,       params->KH_trans);

    /* Pre-compute static transposes */
    arm_mat_trans_f32(&params->F_matrix, &params->F_trans_matrix);
    arm_mat_trans_f32(&params->H_matrix, &params->H_trans_matrix);
}

/* ============================================================
 *  KALMAN_Calc_Acceleration
 *  Torque back-calculation from applied voltage and Kalman velocity.
 *  alpha = ( kt*(V - kt*omega)/R  -  B*omega ) / J
 * ============================================================ */
float32_t KALMAN_Calc_Acceleration(KALMAN_Multi_Model_Params *params, float32_t voltage)
{
    const float32_t omega   = params->X[1];        /* Kalman velocity estimate [rad/s] */
    const float32_t kt      = params->motor->kt;
    const float32_t R       = params->motor->R;
    const float32_t B       = params->motor->B;
    const float32_t J       = params->motor->J;

    float32_t current = (voltage - kt * omega) / R; /* back-EMF current  [A]      */
    float32_t torque  = kt * current;                /* motor torque      [Nm]     */
    float32_t alpha   = (torque - B * omega) / J;    /* angular accel     [rad/s²] */

    params->accel_estimate = alpha;
    return alpha;
}

/* ============================================================
 *  COMPUTE  (called every KALMAN_FREQ seconds)
 * ============================================================ */
void KALMAN_Multi_Model_Compute(KALMAN_Multi_Model_Params* params,
                                float32_t measurement,
                                float32_t input)
{
    params->Z[0] = measurement;
    params->u    = input;

    /* ----------------------------------------------------------
     * Step 1: State Extrapolation   X_new = F*X + G*u
     * ---------------------------------------------------------- */
    arm_mat_mult_f32(&params->F_matrix, &params->X_matrix, &params->FX_matrix);    // FX  = F * X
    arm_mat_scale_f32(&params->G_matrix, params->u, &params->Gu_matrix);            // Gu  = G * u
    arm_mat_add_f32(&params->FX_matrix, &params->Gu_matrix, &params->X_new_matrix); // X_new = FX + Gu
    for (int i = 0; i < STATE_SIZE; i++) { params->X[i] = params->X_new[i]; }

    /* ----------------------------------------------------------
     * Step 2: Covariance Extrapolation   P = F*P*F^T + Q
     * ---------------------------------------------------------- */
    arm_mat_mult_f32(&params->F_matrix,      &params->P_matrix,        &params->FP_matrix);
    arm_mat_mult_f32(&params->FP_matrix,     &params->F_trans_matrix,  &params->FPF_trans_matrix);
    arm_mat_add_f32 (&params->FPF_trans_matrix, &params->Q_matrix,     &params->P_matrix);

    /* ----------------------------------------------------------
     * Step 3: Kalman Gain   K = P*H^T * inv(H*P*H^T + R)
     * ---------------------------------------------------------- */
    arm_mat_mult_f32(&params->H_matrix,   &params->P_matrix,       &params->HP_matrix);
    arm_mat_mult_f32(&params->HP_matrix,  &params->H_trans_matrix, &params->HPH_trans_matrix);
    arm_mat_add_f32 (&params->HPH_trans_matrix, &params->R_matrix, &params->HPH_trans_R_matrix);
    arm_mat_mult_f32(&params->P_matrix,   &params->H_trans_matrix, &params->PH_trans_matrix);

#if (MEASUREMENT_SIZE == 1)
    float32_t S = params->HPH_trans_R[0];
    if (S < INV_SINGULAR_THRESHOLD && S > -INV_SINGULAR_THRESHOLD) {
        return;  // singular — skip this update cycle
    }
    params->HPH_trans_R_inv[0] = 1.0f / S;
#else
    if (arm_mat_inverse_f32(&params->HPH_trans_R_matrix,
                            &params->HPH_trans_R_inv_matrix) != ARM_MATH_SUCCESS) {
        return;
    }
#endif

    arm_mat_mult_f32(&params->PH_trans_matrix, &params->HPH_trans_R_inv_matrix, &params->K_matrix);
    arm_mat_trans_f32(&params->K_matrix, &params->K_trans_matrix);

    /* ----------------------------------------------------------
     * Step 4: State Update   X = X + K*(Z - H*X)
     * ---------------------------------------------------------- */
    arm_mat_mult_f32(&params->H_matrix,    &params->X_matrix,    &params->HX_matrix);
    arm_mat_sub_f32 (&params->Z_matrix,    &params->HX_matrix,   &params->Z_HX_matrix);
    arm_mat_mult_f32(&params->K_matrix,    &params->Z_HX_matrix, &params->KZ_HX_matrix);
    arm_mat_add_f32 (&params->X_matrix,    &params->KZ_HX_matrix, &params->X_new_matrix);
    for (int i = 0; i < STATE_SIZE; i++) { params->X[i] = params->X_new[i]; }

    /* ----------------------------------------------------------
     * Step 5: Covariance Update   P = (I-K*H)*P*(I-K*H)^T + K*R*K^T
     * ---------------------------------------------------------- */
    arm_mat_mult_f32(&params->K_matrix,    &params->H_matrix,         &params->KH_matrix);
    arm_mat_sub_f32 (&params->Iden_matrix, &params->KH_matrix,        &params->IKH_matrix);
    arm_mat_trans_f32(&params->IKH_matrix,                            &params->IKH_trans_matrix);
    arm_mat_mult_f32(&params->IKH_matrix,  &params->P_matrix,         &params->IKHP_matrix);
    arm_mat_mult_f32(&params->IKHP_matrix, &params->IKH_trans_matrix, &params->IKHPIKHP_trans_matrix);
    arm_mat_mult_f32(&params->K_matrix,    &params->R_matrix,         &params->KR_matrix);
    arm_mat_mult_f32(&params->KR_matrix,   &params->K_trans_matrix,   &params->KRK_trans_matrix);
    arm_mat_add_f32 (&params->IKHPIKHP_trans_matrix, &params->KRK_trans_matrix, &params->P_matrix);
}
