#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include "arm_math.h" /* CMSIS-DSP Library */
#include <math.h>
#include "BaseSystem.h"
#include "cascade.h"
#include "kalman.h"
#include "elec_cabient.h"


/* FIX: Changed semicolons to commas for valid C syntax */
typedef enum{
    Trust_Joystick = 0,
    Trust_Basesystem = 1
} ELECTRIC_CABIENT_COMMAND;

typedef enum{
    STATE_WAITING_COMMAND = 0,
    STATE_HOMING = 1,
    STATE_RUNNING = 2
} STATE_MACHINE;

typedef struct
{
    STATE_MACHINE cur_state;
    ELECTRIC_CABIENT_COMMAND trust;
    float32_t home_pos;              /* joystick-set home — used for Go Home trajectory    */
    float32_t basesystem_home_pos;   /* BaseSystem SET_HOME snapshot — origin for BS coords */
    float32_t cur_pos;
    float32_t cur_pos_degree;   /* current position in degrees relative to home */
    int16_t   cur_hole_index;   /* current hole index 0-71 relative to home     */
} SYSTEM_STATE;


typedef struct
{
    TIM_HandleTypeDef *htim_clk;
    TIM_HandleTypeDef *htim_encoder;
    float32_t encoder_data;
    float32_t encoder_degree;
    float32_t encoder_rad;
    float32_t encoder_velo_rad;
    float32_t encoder_prev_data;
    float32_t encoder_TIM_detect_1;
    float32_t encoder_TIM_detect_2;
    float32_t encoder_TIM_diff;
    int32_t   wrap_counter;
} Encoder;

void SYSTEM_STATE_Encoder_Init(TIM_HandleTypeDef *htim_clk, TIM_HandleTypeDef *htim_encoder, Encoder *encoder);
void SYSTEM_STATE_Encoder_Compute(Encoder *encoder);
void PWM(float32_t voltage, float32_t direct_add);

/* Signed voltage PWM() actually applied last call (control + friction, after
 * clamp and off-floor). Feed THIS to the Kalman filter as the true input u. */
extern float32_t vout_applied;

typedef struct
{
    uint16_t instant_detect;
    uint16_t first_detect;
    uint16_t second_detect;
    uint16_t reference_counter;
    uint8_t  flag_ready;
    uint8_t  state_detection;
    int16_t  diff_detect;
    float32_t vout;
    int proximity_flag;
} Proximity;

void SYSTEM_STATE_Homing(Proximity *prox);

/* One-shot init for the system_state subsystem: starts the clock/encoder/PWM
 * timers, binds + seeds the encoder, clears the proximity/homing flags, and
 * sets the state-machine defaults. Call once at startup. */
void SYSTEM_STATE_Init(SYSTEM_STATE *state, Encoder *encoder, Proximity *prox,
                       TIM_HandleTypeDef *htim_clk,
                       TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_pwm);

/* Run one homing step from the main loop. While flag_ready==0 it drives the
 * homing sweep; on flag_ready==1 it seeds the encoder reference + Kalman state,
 * starts the control ISR (htim_isr), and latches flag_ready=2. Returns 1 once
 * homing is complete, 0 while still homing. */
uint8_t SYSTEM_STATE_HomingStep(Proximity *prox, Encoder *encoder,
                                KALMAN_Multi_Model_Params *kalman,
                                SYSTEM_STATE *state,
                                TIM_HandleTypeDef *htim_isr);

/* Bypass the homing sweep: take the CURRENT shaft position as 0 deg / the
 * reference, seed the Kalman state, start the control ISR, and latch done.
 * Use this instead of SYSTEM_STATE_HomingStep to home-in-place immediately. */
void SYSTEM_STATE_SetHomeHere(Proximity *prox, Encoder *encoder,
                              KALMAN_Multi_Model_Params *kalman,
                              SYSTEM_STATE *state,
                              TIM_HandleTypeDef *htim_isr);
/* NEW: Main state machine execution function */

float32_t SYSTEM_STATE_convert_degree2rad(float32_t Deg);
float32_t SYSTEM_STATE_convert_rad2degree(float32_t rad);
#endif /* SYSTEM_STATE_H */
