#include "kalman.h"
#define KALMAN_FREQ 0.001f

void KALMAN_1D_Init(KALMAN_1D_Params* params, float32_t process_noise, float32_t measurement_noise){
   params->x = 0;
   params->p = 1.0f;
   params->q = process_noise;
   params->p += params->q;
   params->r = measurement_noise;
}

void KALMAN_1D_Compute(KALMAN_1D_Params* params, float32_t Input){
   // Predict
   params->x += 0;
   params->p += params->q;

   // Update
   params->K = params->p / (params->p + params->r);
   params->x = params->x + params->K * (Input - params->x);
   params->p = (1 - params->K) * params->p;

   params->z = Input;
   params->position = params->x;
}

void KALMAN_Multi_Velocity_Init(KALMAN_Multi_Velocity_Params* params, float32_t process_noise, float32_t measurement_noise){
   params->R[0] = measurement_noise;

   // Copy temp arrays to struct arrays using loops
   float32_t init_guess_state[2] = {1.0f,1.0f};// X
   for(int i=0; i<2; i++) params->X[i] = init_guess_state[i];

   float32_t init_guess_variance[4] = {0.0f,0.0f,0.0f,0.0f}; //P
   for(int i=0; i<4; i++) params->P[i] = init_guess_variance[i];

   float32_t process_noise_matrix[4] = {0.00000000000025f, 0.00000000033f,
       0.00000000033f, 0.000001}; //Q
   for(int i=0; i<4; i++) params->Q[i] = process_noise_matrix[i] * process_noise;

   float32_t transis_matrix[4] = {1.0f,KALMAN_FREQ,0,1}; // F
   for(int i=0; i<4; i++) params->F[i] = transis_matrix[i];

   float32_t observation_matrix[2] = {1,0}; // H
   for(int i=0; i<2; i++) params->H[i] = observation_matrix[i];

   // Init all matrices
   arm_mat_init_f32(&params->X_matrix, 2, 1, params->X);
   arm_mat_init_f32(&params->P_matrix, 2, 2, params->P);
   arm_mat_init_f32(&params->Q_matrix, 2, 2, params->Q);
   arm_mat_init_f32(&params->F_matrix, 2, 2, params->F);
   arm_mat_init_f32(&params->H_matrix, 1, 2, params->H);
   arm_mat_init_f32(&params->R_matrix, 1, 1, params->R);
   arm_mat_init_f32(&params->K_matrix, 2, 1, params->K);
   arm_mat_init_f32(&params->Z_matrix, 1, 1, params->Z);

   //step 2
   arm_mat_init_f32(&params->FP_matrix, 2, 2, params->FP);
   arm_mat_init_f32(&params->FPF_trans_matrix, 2, 2, params->FPF_trans);
   //step3
   arm_mat_init_f32(&params->HP_matrix, 1, 2, params->HP);
   arm_mat_init_f32(&params->HPH_trans_matrix, 1, 1, params->HPH_trans);
   arm_mat_init_f32(&params->HPH_trans_R_matrix, 1, 1, params->HPH_trans_R);
   arm_mat_init_f32(&params->PH_trans_matrix, 2, 1, params->PH_trans);
   //step4
   arm_mat_init_f32(&params->HX_matrix, 1, 1, params->HX);
   arm_mat_init_f32(&params->Z_HX_matrix, 1, 1, params->Z_HX);
   arm_mat_init_f32(&params->KZ_HX_matrix, 2, 1, params->KZ_HX);
   //step 5
   arm_mat_init_f32(&params->KH_matrix, 2, 2, params->KH);
   arm_mat_init_f32(&params->KH_trans_matrix, 2, 2, params->KH_trans);
   arm_mat_init_f32(&params->KHP_matrix, 2, 2, params->KHP);
   arm_mat_init_f32(&params->KHPKH_trans_matrix, 2, 2, params->KHPKH_trans);
   arm_mat_init_f32(&params->KR_matrix, 2, 1, params->KR);
   arm_mat_init_f32(&params->KRK_trans_matrix, 2, 2, params->KRK_trans);

   // Transposes
   arm_mat_init_f32(&params->F_trans_matrix, 2, 2, params->F_trans);
   arm_mat_init_f32(&params->H_trans_matrix, 2, 1, params->H_trans);
   arm_mat_trans_f32(&params->F_matrix, &params->F_trans_matrix);
   arm_mat_trans_f32(&params->H_matrix, &params->H_trans_matrix);
   arm_mat_init_f32(&params->K_trans_matrix, 1, 2, params->K_trans);

}


void KALMAN_Multi_Velocity_Compute(KALMAN_Multi_Velocity_Params* params, float32_t measurement){
   params->Z[0] = measurement;
   arm_mat_init_f32(&params->Z_matrix, 1, 1, params->Z);  // Update Z matrix

   // Step 1: State Extrapolation X = F * X
   arm_mat_mult_f32(&params->F_matrix, &params->X_matrix, &params->X_matrix);

   // Step 2: State Covariance Extrapolation P = F * P * F^T + Q
   arm_mat_mult_f32(&params->F_matrix, &params->P_matrix, &params->FP_matrix);  // FP = F * P
   arm_mat_mult_f32(&params->FP_matrix, &params->F_trans_matrix, &params->FPF_trans_matrix);  // FPF_trans = FP * F_trans
   arm_mat_add_f32(&params->FPF_trans_matrix, &params->Q_matrix, &params->P_matrix);  // P = FPF_trans + Q

   // Step 3: Kalman Gain K = P * H^T * inv(H * P * H^T + R)
   arm_mat_mult_f32(&params->H_matrix, &params->P_matrix, &params->HP_matrix);  // HP = H * P
   arm_mat_mult_f32(&params->HP_matrix, &params->H_trans_matrix, &params->HPH_trans_matrix);  // HPH_trans = HP * H_trans
   arm_mat_add_f32(&params->HPH_trans_matrix, &params->R_matrix, &params->HPH_trans_R_matrix);  // HPH_trans_R = HPH_trans + R
   arm_mat_mult_f32(&params->P_matrix, &params->H_trans_matrix, &params->PH_trans_matrix);  // PH_trans = P * H_trans
   arm_mat_scale_f32(&params->HPH_trans_R_matrix, 1.0f / params->HPH_trans_R[0], &params->K_matrix);  // K = PH_trans / HPH_trans_R (scalar inv)
   arm_mat_mult_f32(&params->PH_trans_matrix, &params->HPH_trans_R_matrix, &params->K_matrix);  // Simplified K update

   // Step 4: State Update X = X + K * (Z - H * X)
   arm_mat_mult_f32(&params->H_matrix, &params->X_matrix, &params->HX_matrix);  // HX = H * X
   arm_mat_sub_f32(&params->Z_matrix, &params->HX_matrix, &params->Z_HX_matrix);  // Z_HX = Z - HX
   arm_mat_mult_f32(&params->K_matrix, &params->Z_HX_matrix, &params->KZ_HX_matrix);  // KZ_HX = K * Z_HX
   arm_mat_add_f32(&params->X_matrix, &params->KZ_HX_matrix, &params->X_matrix);  // X = X + KZ_HX


   // Step 5: Covariance Update P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
   // Compute KH = K * H again for Step 5
   arm_mat_mult_f32(&params->K_matrix, &params->H_matrix, &params->KH_matrix);  // KH = K * H

   // Compute I - KH: modify KH in place
   params->KH[0] = 1.0f - params->KH[0];
   params->KH[1] = 0.0f - params->KH[1];
   params->KH[2] = 0.0f - params->KH[2];
   params->KH[3] = 1.0f - params->KH[3];
   arm_mat_init_f32(&params->KH_matrix, 2, 2, params->KH);

   // (I-KH)*P
   arm_mat_mult_f32(&params->KH_matrix, &params->P_matrix, &params->KHP_matrix);
   // (I-KH)*P*(I-KH)^T = KHP * KH^T
   arm_mat_trans_f32(&params->KH_matrix, &params->KH_trans_matrix);
   arm_mat_mult_f32(&params->KHP_matrix, &params->KH_trans_matrix, &params->KHPKH_trans_matrix);
   // K*R*K^T
   arm_mat_mult_f32(&params->K_matrix, &params->R_matrix, &params->KR_matrix);
   arm_mat_mult_f32(&params->KR_matrix, &params->K_trans_matrix, &params->KRK_trans_matrix);
   // P = (I-KH)P(I-KH)^T + KRK^T
   arm_mat_add_f32(&params->KHPKH_trans_matrix, &params->KRK_trans_matrix, &params->P_matrix);

   // Update transposes if needed
   arm_mat_trans_f32(&params->K_matrix, &params->K_trans_matrix);
}

// void KALMAN_Multi_Model_Init(KALMAN_Multi_Velocity_Params* params, float32_t process_noise, float32_t measurement_noise, float32_t mass, float32_t damp, float32_t spring){
//     params->R[0] = measurement_noise;

//    // Copy temp arrays to struct arrays using loops
//    float32_t init_guess_state[2] = {1.0f,1.0f};// X
//    for(int i=0; i<2; i++) params->X[i] = init_guess_state[i];

//    float32_t init_guess_variance[4] = {0.0f,0.0f,0.0f,0.0f}; //P
//    for(int i=0; i<4; i++) params->P[i] = init_guess_variance[i];

//    float32_t process_noise_matrix[4] = {(KALMAN_FREQ*KALMAN_FREQ*KALMAN_FREQ*KALMAN_FREQ)/4, (KALMAN_FREQ*KALMAN_FREQ*KALMAN_FREQ)/3,
//        (KALMAN_FREQ*KALMAN_FREQ*KALMAN_FREQ)/3, KALMAN_FREQ*KALMAN_FREQ}; //Q
//    for(int i=0; i<4; i++) params->Q[i] = process_noise_matrix[i] * params->accel_process_noise;

//    float32_t buffer1 = -spring*KALMAN_FREQ/mass;
//    float32_t buffer2 = 1- (damp*KALMAN_FREQ/mass);
//    float32_t transis_matrix[4] = {1.0f,KALMAN_FREQ,buffer1,buffer2}; // F
//    for(int i=0; i<4; i++) params->F[i] = transis_matrix[i];

//    float32_t observation_matrix[2] = {1,0}; // H
//    for(int i=0; i<2; i++) params->H[i] = observation_matrix[i];

//    // Init all matrices
//    arm_mat_init_f32(&params->X_matrix, 2, 1, params->X);
//    arm_mat_init_f32(&params->P_matrix, 2, 2, params->P);
//    arm_mat_init_f32(&params->Q_matrix, 2, 2, params->Q);
//    arm_mat_init_f32(&params->F_matrix, 2, 2, params->F);
//    arm_mat_init_f32(&params->H_matrix, 1, 2, params->H);
//    arm_mat_init_f32(&params->R_matrix, 1, 1, params->R);
//    arm_mat_init_f32(&params->K_matrix, 2, 1, params->K);
//    arm_mat_init_f32(&params->Z_matrix, 1, 1, params->Z);

//    //step 2
//    arm_mat_init_f32(&params->FP_matrix, 2, 2, params->FP);
//    arm_mat_init_f32(&params->FPF_trans_matrix, 2, 2, params->FPF_trans);
//    //step3
//    arm_mat_init_f32(&params->HP_matrix, 1, 2, params->HP);
//    arm_mat_init_f32(&params->HPH_trans_matrix, 1, 1, params->HPH_trans);
//    arm_mat_init_f32(&params->HPH_trans_R_matrix, 1, 1, params->HPH_trans_R);
//    arm_mat_init_f32(&params->PH_trans_matrix, 2, 1, params->PH_trans);
//    //step4
//    arm_mat_init_f32(&params->HX_matrix, 1, 1, params->HX);
//    arm_mat_init_f32(&params->Z_HX_matrix, 1, 1, params->Z_HX);
//    arm_mat_init_f32(&params->KZ_HX_matrix, 2, 1, params->KZ_HX);
//    //step 5
//    arm_mat_init_f32(&params->KH_matrix, 2, 2, params->KH);
//    arm_mat_init_f32(&params->KH_trans_matrix, 2, 2, params->KH_trans);
//    arm_mat_init_f32(&params->KHP_matrix, 2, 2, params->KHP);
//    arm_mat_init_f32(&params->KHPKH_trans_matrix, 2, 2, params->KHPKH_trans);
//    arm_mat_init_f32(&params->KR_matrix, 2, 1, params->KR);
//    arm_mat_init_f32(&params->KRK_trans_matrix, 2, 2, params->KRK_trans);

//    // Transposes
//    arm_mat_init_f32(&params->F_trans_matrix, 2, 2, params->F_trans);
//    arm_mat_init_f32(&params->H_trans_matrix, 2, 1, params->H_trans);
//    arm_mat_trans_f32(&params->F_matrix, &params->F_trans_matrix);
//    arm_mat_trans_f32(&params->H_matrix, &params->H_trans_matrix);
//    arm_mat_init_f32(&params->K_trans_matrix, 1, 2, params->K_trans);

// }
// void KALMAN_Multi_Model_Compute(KALMAN_Multi_Model_Params* params, float32_t measurement){

// }
