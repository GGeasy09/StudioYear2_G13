#include "system_state.h"
#include "tim.h"
#include "gpio.h"

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

#define ENCODER_COUNTS_PER_REV  8192.0f
#define DEG_PER_REV             360.0f
#define RAD_PER_REV             (2.0f * 3.14159265f)

#define PWM_PERIOD              42500           /* TIM1 ARR value          */
#define VOLTAGE_MAX             12.0f           /* Motor supply rail (V)   */
#define Encoderhome             30000
#define Encoderlimit            16384

/* Signed voltage actually applied by PWM() (control + friction, post-clamp/floor).
 * Read by the Kalman step as the true motor input u. */
float32_t vout_applied = 0.0f;
/* Direction pin: HIGH = forward, LOW = reverse */
#define DIR_PORT   GPIOC
#define DIR_PIN    GPIO_PIN_3

/* -------------------------------------------------------------------------
 * SYSTEM_STATE_Encoder_Compute
 *
 * Converts raw 16-bit encoder counter to degrees and radians.
 * Handles unsigned wrap-around by casting to int16_t so that
 * values above 32767 are interpreted as negative (reverse rotation).
 * ---------------------------------------------------------------------- */




void SYSTEM_STATE_Encoder_Init(TIM_HandleTypeDef *htim_clk, TIM_HandleTypeDef *htim_encoder,Encoder *encoder){
    encoder->htim_clk = htim_clk;
    encoder->htim_encoder = htim_encoder;
}

/* =========================================================================
 * SYSTEM_STATE_Init — one-shot init for the whole system_state subsystem
 * ======================================================================= */
void SYSTEM_STATE_Init(SYSTEM_STATE *state, Encoder *encoder, Proximity *prox,
                       TIM_HandleTypeDef *htim_clk,
                       TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_pwm)
{
    /* 1. Start hardware timers and force the motor off */
    HAL_TIM_Base_Start(htim_clk);
    HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1);
    PWM(0.0f, 0.0f);

    /* 2. Encoder: bind handles, seed counter to home centre, clear tracking */
    SYSTEM_STATE_Encoder_Init(htim_clk, htim_encoder, encoder);
    __HAL_TIM_SET_COUNTER(htim_encoder, Encoderhome);
    encoder->wrap_counter      = 0;
    encoder->encoder_prev_data = 0.0f;   /* current_counts at home = Encoderhome - Encoderhome = 0 */

    /* 3. Proximity / homing flags.
     *    flag_ready = 0 -> not homed yet. The main loop then either runs the
     *    active sweep (SYSTEM_STATE_HomingStep) or homes in place
     *    (SYSTEM_STATE_SetHomeHere), depending on the home_here toggle. */
    prox->flag_ready        = 0;
    prox->state_detection   = 0;
    prox->proximity_flag    = 0;
    prox->reference_counter = Encoderhome;
    prox->vout              = 0.0f;

    /* 4. State-machine defaults */
    state->cur_state      = STATE_WAITING_COMMAND;
    state->cur_pos        = 0.0f;
    state->cur_pos_degree = 0.0f;
}

/* =========================================================================
 * SYSTEM_STATE_HomingStep — one homing iteration, called from the main loop
 * ======================================================================= */
uint8_t SYSTEM_STATE_HomingStep(Proximity *prox, Encoder *encoder,
                                KALMAN_Multi_Model_Params *kalman,
                                SYSTEM_STATE *state,
                                TIM_HandleTypeDef *htim_isr)
{
    if (prox->flag_ready == 0)
    {
        /* Still homing: drive the sweep */
        SYSTEM_STATE_Homing(prox);
        PWM(prox->vout, 0.0f);
        return 0;
    }
    else if (prox->flag_ready == 1)
    {
        /* Homing done: reset encoder counter to reference, seed Kalman, start ISR */
        __HAL_TIM_SET_COUNTER(encoder->htim_encoder, prox->reference_counter);
        encoder->wrap_counter      = 0;
        encoder->encoder_prev_data = 0.0f;
        SYSTEM_STATE_Encoder_Compute(encoder);

        kalman->X[0] = encoder->encoder_rad;
        kalman->X[1] = 0.0f;
        kalman->X[2] = 0.0f;
        kalman->X[3] = 0.0f;

        state->cur_pos        = 0.0f;
        state->cur_pos_degree = SYSTEM_STATE_convert_rad2degree(encoder->encoder_rad);

        PWM(0.0f, 0.0f);
        prox->flag_ready = 2;             /* latch: this block runs only once */
        HAL_TIM_Base_Start_IT(htim_isr);  /* control ISR starts now */
        return 1;
    }

    return 1;   /* flag_ready == 2: already homed */
}

/* =========================================================================
 * SYSTEM_STATE_SetHomeHere — bypass sweep, zero at the current position
 * ======================================================================= */
void SYSTEM_STATE_SetHomeHere(Proximity *prox, Encoder *encoder,
                              KALMAN_Multi_Model_Params *kalman,
                              SYSTEM_STATE *state,
                              TIM_HandleTypeDef *htim_isr)
{
    /* Force the encoder count to the home centre so the CURRENT physical angle
     * reads as 0 rad / 0 deg, and that becomes the reference. */
    __HAL_TIM_SET_COUNTER(encoder->htim_encoder, Encoderhome);
    encoder->wrap_counter      = 0;
    encoder->encoder_prev_data = 0.0f;
    SYSTEM_STATE_Encoder_Compute(encoder);

    kalman->X[0] = encoder->encoder_rad;   /* ~0 */
    kalman->X[1] = 0.0f;
    kalman->X[2] = 0.0f;
    kalman->X[3] = 0.0f;

    state->cur_pos        = 0.0f;
    state->cur_pos_degree = 0.0f;

    prox->reference_counter = Encoderhome;
    prox->flag_ready        = 2;            /* latch: homed, run once */
    PWM(0.0f, 0.0f);
    HAL_TIM_Base_Start_IT(htim_isr);        /* control ISR starts now */
}

void SYSTEM_STATE_Encoder_Compute(Encoder *encoder)
{
    // 1. Log current time from the dedicated clock timer
    encoder->encoder_TIM_detect_1 = (float32_t)__HAL_TIM_GET_COUNTER(encoder->htim_clk);
    
    // 2. Read live counter from the hardware encoder timer
    encoder->encoder_data = (float32_t)__HAL_TIM_GET_COUNTER(encoder->htim_encoder);
    
    // Read raw encoder and apply home offset
    int32_t raw_counts = (int32_t)(int16_t)encoder->encoder_data;
    float32_t current_counts = (float32_t)(Encoderhome - raw_counts);    
    
    // 3. Detect overflow/underflow + spike rejection (mutually exclusive branches)
    float32_t delta_counts = current_counts - encoder->encoder_prev_data;

    if (delta_counts > 32767.0f)
    {
        encoder->wrap_counter--;        // Real hardware wrap (forward → backward)
        delta_counts -= 65536.0f;
    }
    else if (delta_counts < -32768.0f)
    {
        encoder->wrap_counter++;        // Real hardware wrap (backward → forward)
        delta_counts += 65536.0f;
    }
    else if (delta_counts > 50.0f || delta_counts < -50.0f)
    {
        // Noise spike (50 < |delta| < 32767) — freeze position, leave wrap_counter untouched
        current_counts = encoder->encoder_prev_data;
        delta_counts   = 0.0f;
    }
    
    // 4. Calculate CONTINUOUS multi-turn counts
    float32_t continuous_counts = current_counts + (encoder->wrap_counter * 65536.0f);
    
    // Position now adds up continuously
    encoder->encoder_degree = continuous_counts / ENCODER_COUNTS_PER_REV * DEG_PER_REV;
    encoder->encoder_rad    = continuous_counts / ENCODER_COUNTS_PER_REV * RAD_PER_REV;
    
    // 5. Fix time difference (Current - Previous)
    encoder->encoder_TIM_diff = encoder->encoder_TIM_detect_1 - encoder->encoder_TIM_detect_2;
    
    // Handle Timer Rollover (Assuming your clock timer is 32-bit, e.g., TIM2 or TIM5 on STM32G4)
    if(encoder->encoder_TIM_diff < 0.0f){
        encoder->encoder_TIM_diff += 4294967296.0f; 
    }
    
    // 6. Calculate velocity safely
    if(encoder->encoder_TIM_diff > 0.0f){
        float32_t velo = delta_counts / encoder->encoder_TIM_diff;
        // NOTE: Make sure the 1000000.0f matches the frequency of your htim_clk!
        encoder->encoder_velo_rad = velo / ENCODER_COUNTS_PER_REV * RAD_PER_REV * 1000000.0f;
    } else {
        encoder->encoder_velo_rad = 0.0f; 
    }

    // 7. Save current RAW state for the next loop
    encoder->encoder_prev_data = current_counts;
    encoder->encoder_TIM_detect_2 = encoder->encoder_TIM_detect_1;
}

/* -------------------------------------------------------------------------
 * PWM
 *
 * Converts a signed voltage demand (−12 V … +12 V) into a PWM compare
 * value and sets the direction GPIO accordingly.
 *
 *  voltage > 0  → forward  (DIR HIGH)
 *  voltage < 0  → reverse  (DIR LOW), magnitude drives PWM
 *  voltage = 0  → brake    (PWM = 0)
 * ---------------------------------------------------------------------- */

void PWM(float32_t voltage, float32_t direct_add)
{
    /* `direct_add` is now a SIGNED friction-comp voltage (already carries the
     * correct direction and self-tapers to 0 near the setpoint). Combine it with
     * the control voltage BEFORE the direction decision so the kick always lands
     * the right way, then pick the GPIO direction from the resulting sign. */
    voltage += direct_add;

    /* Clamp to rail limits */
    if      (voltage >  VOLTAGE_MAX) voltage =  VOLTAGE_MAX;
    else if (voltage < -VOLTAGE_MAX) voltage = -VOLTAGE_MAX;

    /* Tiny floor: below this the combined command is essentially zero -> brake.
     * The friction term already fades out near target, so this can be small and
     * the motor no longer stops short of the setpoint. */
    const float32_t PWM_VOLTAGE_FLOOR = 0.05f;   /* volts */
    float32_t vmag = (voltage < 0.0f) ? -voltage : voltage;
    if (vmag < PWM_VOLTAGE_FLOOR)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   /* hold off */
        vout_applied = 0.0f;   /* motor off -> 0 V actually applied */
        return;
    }

    /* Direction from the sign of the combined command */
    if (voltage >= 0.0f)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
    else
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);

    vout_applied = voltage;   /* signed voltage actually applied (control + friction) */

    /* Scale magnitude to timer counts: compare = period * |V| / V_max */
    uint16_t compare = (uint16_t)(PWM_PERIOD * vmag / VOLTAGE_MAX);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}

void SYSTEM_STATE_Homing(Proximity *prox)
{
    // Static variable persists its value across multiple function calls
    static uint32_t backoff_start_time = 0;

    switch (prox->state_detection)
    {
        case 0:
            prox->vout = 3.0;

            if (prox->proximity_flag == 1)
            {
                prox->first_detect = prox->instant_detect;

                // Command the motor to reverse aggressively
                prox->vout = 3.0;

                // Record the exact millisecond we started backing off
                backoff_start_time = HAL_GetTick();

                // Advance to the new non-blocking "wait" state
                prox->state_detection = 99;
            }
            break;

        case 99:
            /* This is the Non-Blocking Timestamp State */
            prox->vout = 3.0;


            // Check if 500 milliseconds have passed since we recorded the start time
            if ((HAL_GetTick() - backoff_start_time) >= 750)
            {
                // Time is up! Clear the interrupt flag so we don't double-trigger
                prox->proximity_flag = 0;

                // Advance to the next real scan phase
                prox->state_detection = 1;
            }
            break;

        case 1:
            prox->vout = -1.0;

            if (prox->proximity_flag == 1)
            {
                prox->second_detect = prox->instant_detect;
                prox->state_detection = 2; // Advance to math phase
                prox->proximity_flag = 0;
            }
            break;

        case 2: // Using an explicit case 2 is safer than 'default'
            // Calculate center
            prox->diff_detect = (prox->first_detect - prox->second_detect)/2;
            prox->reference_counter = Encoderhome - prox->diff_detect - 40;
            prox->flag_ready = 1;
            __HAL_TIM_SET_COUNTER(&htim3,prox->reference_counter);
            prox->vout = 0.0;
            break;

        default:
            prox->vout = 0.0;
            break;
    }
}

float32_t SYSTEM_STATE_convert_degree2rad(float32_t Deg){
    return Deg*0.0174533f;
}
float32_t SYSTEM_STATE_convert_rad2degree(float32_t rad){
    return rad*57.2958f;
}

