#pragma once
#include <arm_math.h>

//Kalman 1D
//Kalman 2D Velocity
//Kalman Spring Model

//Kalman 1D
typedef struct KALMAN_1D{
   float32_t x;
   float32_t K;
   float32_t z;
   float32_t p;
   float32_t q;
   float32_t r;
   float32_t position;
}KALMAN_1D_Params;

void KALMAN_1D_Init(KALMAN_1D_Params* params, float32_t process_noise, float32_t measurement_noise);
void KALMAN_1D_Compute(KALMAN_1D_Params* params, float32_t Input);

//Kalman 2D
typedef struct KALMAN_Multi_Velocity{
   float32_t X[2];                 arm_matrix_instance_f32 X_matrix;
   float32_t F[4];                 arm_matrix_instance_f32 F_matrix;
   float32_t F_trans[4];           arm_matrix_instance_f32 F_trans_matrix;
   float32_t Q[4];                 arm_matrix_instance_f32 Q_matrix;
   float32_t P[4];                 arm_matrix_instance_f32 P_matrix;
   float32_t H[2];                 arm_matrix_instance_f32 H_matrix;
   float32_t H_trans[2];           arm_matrix_instance_f32 H_trans_matrix;
   float32_t Z[1];                 arm_matrix_instance_f32 Z_matrix;
   float32_t R[1];                 arm_matrix_instance_f32 R_matrix;
   float32_t K[2];                 arm_matrix_instance_f32 K_matrix;
   float32_t K_trans[2];           arm_matrix_instance_f32 K_trans_matrix;


   //Step 1 State Extrapolation X = FX
   //X = FX

   //Step 2 State Covariance Extrapolate P = FPF_trans+Q
   float32_t FP[4];                arm_matrix_instance_f32 FP_matrix;
   float32_t FPF_trans[4];         arm_matrix_instance_f32 FPF_trans_matrix;
   //P = FPF_trans+Q

   //Step 3 KALMAN Gain Update K = PH_trans(HPH_trans+R)inverse
   float32_t HP[2];                arm_matrix_instance_f32 HP_matrix;
   float32_t HPH_trans[1];         arm_matrix_instance_f32 HPH_trans_matrix;
   float32_t HPH_trans_R[1];       arm_matrix_instance_f32 HPH_trans_R_matrix;
   float32_t PH_trans[2];          arm_matrix_instance_f32 PH_trans_matrix;
   //K = PH_trans*HPH_trans_R

   //Step 4 State Update Calculate Matrix X = X+K(Z-HX)
   float32_t HX[1]; //HX
                                   arm_matrix_instance_f32 HX_matrix;
   float32_t Z_HX[1]; //Z-HX
                                   arm_matrix_instance_f32 Z_HX_matrix;
   float32_t KZ_HX[2]; //K(Z-HX)
                                   arm_matrix_instance_f32 KZ_HX_matrix;
   //X = X+KZ_HX

   //Step 5 Covarince Update P = (I-KH)P(I-KH)transe+KRK_transe
   float32_t KH[4];                arm_matrix_instance_f32 KH_matrix;
   //KH = I-KH
   float32_t KH_trans[4];          arm_matrix_instance_f32 KH_trans_matrix;
   float32_t KHP[4];               arm_matrix_instance_f32 KHP_matrix;
   float32_t KHPKH_trans[4];       arm_matrix_instance_f32 KHPKH_trans_matrix;
   float32_t KR[2];                arm_matrix_instance_f32 KR_matrix;
   float32_t KRK_trans[4];         arm_matrix_instance_f32 KRK_trans_matrix;
   //P = KHPKH_trans + KRK_trans
}KALMAN_Multi_Velocity_Params;

void KALMAN_Multi_Velocity_Init(KALMAN_Multi_Velocity_Params* params, float32_t process_noise, float32_t measurement_noise);
void KALMAN_Multi_Velocity_Compute(KALMAN_Multi_Velocity_Params* params, float32_t measurement);

//Kalman 2D Model with spring
//typedef struct KALMAN_Multi_Model{
//  float32_t X[2];                 arm_matrix_instance_f32 X_matrix;
//  float32_t F[4];                 arm_matrix_instance_f32 F_matrix;
//  float32_t F_trans[4];           arm_matrix_instance_f32 F_trans_matrix;
//  float32_t Q[4];                 arm_matrix_instance_f32 Q_matrix;
//  float32_t P[4];                 arm_matrix_instance_f32 P_matrix;
//  float32_t H[2];                 arm_matrix_instance_f32 H_matrix;
//  float32_t H_trans[2];           arm_matrix_instance_f32 H_trans_matrix;
//  float32_t Z[1];                 arm_matrix_instance_f32 Z_matrix;
//  float32_t R[1];                 arm_matrix_instance_f32 R_matrix;
//  float32_t K[2];                 arm_matrix_instance_f32 K_matrix;
//  float32_t K_trans[2];           arm_matrix_instance_f32 K_trans_matrix;
//  float32_t G[2];                 arm_matrix_instance_f32 G;
//  float32_t u;
//
//
//  //Step 1 State Extrapolation X = FX + Gu
//  float32_t FX[2];                arm_matrix_instance_f32 FX;
//  float32_t Gu[2];                arm_matrix_instance_f32 Gu;
//  //X = FX + Gu
//
//  //Step 2 State Covariance Extrapolate P = FPF_trans+Q
//  float32_t FP[4];                arm_matrix_instance_f32 FP_matrix;
//  float32_t FPF_trans[4];         arm_matrix_instance_f32 FPF_trans_matrix;
//  //P = FPF_trans+Q
//
//  //Step 3 KALMAN Gain Update K = PH_trans(HPH_trans+R)inverse
//  float32_t HP[2];                arm_matrix_instance_f32 HP_matrix;
//  float32_t HPH_trans[1];         arm_matrix_instance_f32 HPH_trans_matrix;
//  float32_t HPH_trans_R[1];       arm_matrix_instance_f32 HPH_trans_R_matrix;
//  float32_t PH_trans[2];          arm_matrix_instance_f32 PH_trans_matrix;
//  //K = PH_trans*HPH_trans_R
//
//  //Step 4 State Update Calculate Matrix X = X+K(Z-HX)
//  float32_t HX[1]; //HX
//                                  arm_matrix_instance_f32 HX_matrix;
//  float32_t Z_HX[1]; //Z-HX
//                                  arm_matrix_instance_f32 Z_HX_matrix;
//  float32_t KZ_HX[2]; //K(Z-HX)
//                                  arm_matrix_instance_f32 KZ_HX_matrix;
//  //X = X+KZ_HX
//
//  //Step 5 Covarince Update P = (I-KH)P(I-KH)transe+KRK_transe
//  float32_t KH[4];                arm_matrix_instance_f32 KH_matrix;
//  //KH = I-KH
//  float32_t KH_trans[4];          arm_matrix_instance_f32 KH_trans_matrix;
//  float32_t KHP[4];               arm_matrix_instance_f32 KHP_matrix;
//  float32_t KHPKH_trans[4];       arm_matrix_instance_f32 KHPKH_trans_matrix;
//  float32_t KR[2];                arm_matrix_instance_f32 KR_matrix;
//  float32_t KRK_trans[4];         arm_matrix_instance_f32 KRK_trans_matrix;
//  //P = KHPKH_trans + KRK_trans
//}KALMAN_Multi_Model_Params;
//
//void KALMAN_Multi_Model_Init(KALMAN_Multi_Velocity_Params* params, float32_t process_noise, float32_t measurement_noise);
//void KALMAN_Multi_Model_Compute(KALMAN_Multi_Model_Params* params, float32_t measurement);
