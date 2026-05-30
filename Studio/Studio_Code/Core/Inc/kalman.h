#ifndef KALMAN_H
#define KALMAN_H

#include <arm_math.h>
#include "motor_params.h"

#define STATE_SIZE       4
#define MEASUREMENT_SIZE 1
#define INPUT_SIZE       1

typedef struct {
    float32_t Process_Noise;

    /* Motor model reference (set by Init, used by Calc_Acceleration) */
    MOTOR_PARAMS *motor;

    /* Result of the last KALMAN_Calc_Acceleration() call [rad/s^2] */
    float32_t accel_estimate;

    /* ----- Core Kalman matrices ----- */
    float32_t X[STATE_SIZE];                            arm_matrix_instance_f32 X_matrix;
    float32_t F[STATE_SIZE * STATE_SIZE];               arm_matrix_instance_f32 F_matrix;
    float32_t F_trans[STATE_SIZE * STATE_SIZE];         arm_matrix_instance_f32 F_trans_matrix;
    float32_t Q[STATE_SIZE * STATE_SIZE];               arm_matrix_instance_f32 Q_matrix;
    float32_t P[STATE_SIZE * STATE_SIZE];               arm_matrix_instance_f32 P_matrix;
    float32_t H[MEASUREMENT_SIZE * STATE_SIZE];         arm_matrix_instance_f32 H_matrix;
    float32_t H_trans[STATE_SIZE * MEASUREMENT_SIZE];   arm_matrix_instance_f32 H_trans_matrix;
    float32_t Z[MEASUREMENT_SIZE];                      arm_matrix_instance_f32 Z_matrix;
    float32_t R[MEASUREMENT_SIZE * MEASUREMENT_SIZE];   arm_matrix_instance_f32 R_matrix;
    float32_t K[STATE_SIZE * MEASUREMENT_SIZE];         arm_matrix_instance_f32 K_matrix;
    float32_t K_trans[MEASUREMENT_SIZE * STATE_SIZE];   arm_matrix_instance_f32 K_trans_matrix;
    float32_t G[STATE_SIZE * INPUT_SIZE];               arm_matrix_instance_f32 G_matrix;
    float32_t u;
    float32_t Iden[STATE_SIZE * STATE_SIZE];            arm_matrix_instance_f32 Iden_matrix;

    /* ----- Step 1: State Extrapolation  X_new = F*X + G*u ----- */
    float32_t FX[STATE_SIZE];                           arm_matrix_instance_f32 FX_matrix;
    float32_t Gu[STATE_SIZE];                           arm_matrix_instance_f32 Gu_matrix;
    float32_t X_new[STATE_SIZE];                        arm_matrix_instance_f32 X_new_matrix;

    /* ----- Step 2: Covariance Extrapolation  P = F*P*F^T + Q ----- */
    float32_t FP[STATE_SIZE * STATE_SIZE];              arm_matrix_instance_f32 FP_matrix;
    float32_t FPF_trans[STATE_SIZE * STATE_SIZE];       arm_matrix_instance_f32 FPF_trans_matrix;

    /* ----- Step 3: Kalman Gain  K = P*H^T * inv(H*P*H^T + R) ----- */
    float32_t HP[MEASUREMENT_SIZE * STATE_SIZE];                    arm_matrix_instance_f32 HP_matrix;
    float32_t HPH_trans[MEASUREMENT_SIZE * MEASUREMENT_SIZE];       arm_matrix_instance_f32 HPH_trans_matrix;
    float32_t HPH_trans_R[MEASUREMENT_SIZE * MEASUREMENT_SIZE];     arm_matrix_instance_f32 HPH_trans_R_matrix;
    float32_t HPH_trans_R_inv[MEASUREMENT_SIZE * MEASUREMENT_SIZE]; arm_matrix_instance_f32 HPH_trans_R_inv_matrix;
    float32_t PH_trans[STATE_SIZE * MEASUREMENT_SIZE];              arm_matrix_instance_f32 PH_trans_matrix;

    /* ----- Step 4: State Update  X = X + K*(Z - H*X) ----- */
    float32_t HX[MEASUREMENT_SIZE];                     arm_matrix_instance_f32 HX_matrix;
    float32_t Z_HX[MEASUREMENT_SIZE];                   arm_matrix_instance_f32 Z_HX_matrix;
    float32_t KZ_HX[STATE_SIZE];                        arm_matrix_instance_f32 KZ_HX_matrix;

    /* ----- Step 5: Covariance Update  P = (I-K*H)*P*(I-K*H)^T + K*R*K^T ----- */
    float32_t KH[STATE_SIZE * STATE_SIZE];              arm_matrix_instance_f32 KH_matrix;
    float32_t IKH[STATE_SIZE * STATE_SIZE];             arm_matrix_instance_f32 IKH_matrix;
    float32_t IKH_trans[STATE_SIZE * STATE_SIZE];       arm_matrix_instance_f32 IKH_trans_matrix;
    float32_t IKHP[STATE_SIZE * STATE_SIZE];            arm_matrix_instance_f32 IKHP_matrix;
    float32_t IKHPIKHP_trans[STATE_SIZE * STATE_SIZE];  arm_matrix_instance_f32 IKHPIKHP_trans_matrix;
    float32_t KR[STATE_SIZE * MEASUREMENT_SIZE];        arm_matrix_instance_f32 KR_matrix;
    float32_t KRK_trans[STATE_SIZE * STATE_SIZE];       arm_matrix_instance_f32 KRK_trans_matrix;
    float32_t KH_trans[STATE_SIZE * STATE_SIZE];        arm_matrix_instance_f32 KH_trans_matrix;

    /* ----- Legacy fields (unused — kept for ABI compat) ----- */
    float32_t KHP[STATE_SIZE * STATE_SIZE];             arm_matrix_instance_f32 KHP_matrix;
    float32_t KHPKH_trans[STATE_SIZE * STATE_SIZE];     arm_matrix_instance_f32 KHPKH_trans_matrix;

} KALMAN_Multi_Model_Params;

/* =========================================================================
 * Core Kalman functions
 * ========================================================================= */
void KALMAN_Multi_Model_Init(KALMAN_Multi_Model_Params *params,
                             float32_t process_noise,
                             float32_t disturbance_noise,
                             float32_t measurement_noise,
                             MOTOR_PARAMS *parameter);

void KALMAN_Multi_Model_Compute(KALMAN_Multi_Model_Params *params,
                                float32_t measurement,
                                float32_t input);

/* =========================================================================
 * KALMAN_Calc_Acceleration
 *
 * Estimates angular acceleration [rad/s^2] via torque back-calculation:
 *   i = (voltage - kt * omega) / R     back-EMF current [A]
 *   T = kt * i                          motor torque     [Nm]
 *   a = (T - B * omega) / J             acceleration     [rad/s^2]
 *
 * Reads:  params->X[1] (Kalman velocity), params->motor (set during Init)
 * Writes: params->accel_estimate
 * Returns: acceleration [rad/s^2]
 * ========================================================================= */
float32_t KALMAN_Calc_Acceleration(KALMAN_Multi_Model_Params *params, float32_t voltage);

#endif /* KALMAN_H */
