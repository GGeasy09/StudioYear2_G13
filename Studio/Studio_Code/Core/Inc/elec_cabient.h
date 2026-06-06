#ifndef JOY_STICK_H
#define JOY_STICK_H
#include <stdint.h>
#include "arm_math.h" /* CMSIS-DSP Library */
#include "gpio.h"

/* ===========================================================================
 * Gripper transport selection
 *   0 = Direct GPIO (default, original behaviour)
 *   1 = CAN bus via Protocol Spec v1.0.1 (gripper_can.h / gripper_can.c)
 * =========================================================================*/
#ifndef GRIPPER_USE_CAN
#define GRIPPER_USE_CAN  1
#endif
typedef struct
{
    /* --- Logical States (1 = Pressed, 0 = Released) --- */
    uint8_t mode;               /* PA15: Mode switch */
    uint8_t btn_black;          /* PA9 : Black button */
    uint8_t btn_red;            /* PA8 : Red button */
    uint8_t btn_white_left;     /* PC1 : Left White button */
    uint8_t btn_white_right;    /* PB1 : Right White button (Note: hardware link triggers left too) */
    uint8_t btn_blue;           /* PB2 : Blue button */
    uint8_t btn_yellow;         /* PB9 : Yellow button */
    uint8_t soft_stop;          /* PB7 : Soft-stop safety line */

    /* --- Raw Hardware Line States (Direct HAL Read: 0 = Low, 1 = High) --- */
    uint8_t raw_mode;
    uint8_t raw_btn_black;
    uint8_t raw_btn_red;
    uint8_t raw_btn_white_left;
    uint8_t raw_btn_white_right;
    uint8_t raw_btn_blue;
    uint8_t raw_btn_yellow;
    uint8_t raw_btn_soft_stop;
} Joystick;
/* Function prototype to add at the bottom of system_state.h */
void SYSTEM_STATE_Joystick_Update(Joystick *joy);

typedef struct
{
    uint8_t relay1;   /* PC8: 1 = Power ON / Emer OFF,  0 = Emer ON / Power OFF */
    uint8_t relay2;   /* PC9: 1 = Manual ON / Auto OFF,  0 = Auto ON  / Manual OFF */
} Pilot_ramp;

/* relay1: 1=Power ON/Emer OFF  0=Emer ON/Power OFF */
void PilotRamp_SetPower(Pilot_ramp *ramp, uint8_t on);

/* relay2: 1=Auto ON/Manual OFF  0=Manual ON/Auto OFF */
void PilotRamp_SetAuto(Pilot_ramp *ramp, uint8_t on);

/* ===========================================================================
 * Gripper
 * =========================================================================*/
#define GRIPPER_PHASE_DELAY_MS  750U   /* default wait between pick/place actions */

typedef struct
{
    /* --- Output states (raw relay command) --- */
    uint8_t grip_open;   /* PC4: Relay3 — 0=open,  1=close */
    uint8_t grip_up;     /* PC5: Relay4 — 0=down,  1=up    */

    /* --- Sensor states (updated by Gripper_ReadState) --- */
    uint8_t is_full_open;   /* PC6:  1 = fully open  */
    uint8_t is_full_close;  /* PB12: 1 = fully closed */
    uint8_t is_up;          /* PC2:  1 = up, 0 = down */

    /* --- Internal phase for non-blocking pick/place --- */
    uint8_t  action_phase;
    uint32_t delay_start;    /* timestamp used by caller's own timeout  */
    uint32_t phase_start;    /* timestamp for internal phase delay       */
    uint32_t action_delay_ms;/* wait between actions — 0 = use default  */
} Gripper;

/* Control — raw relay command values */
void Gripper_SetOpen  (Gripper *g, uint8_t cmd);   /* 0=open, 1=close */
void Gripper_SetUp    (Gripper *g, uint8_t cmd);   /* 0=down, 1=up    */

/* Read all sensor states into struct */
void Gripper_ReadState(Gripper *g);

/* Non-blocking pick/place — call each main loop tick, returns 1 when done */
uint8_t Gripper_Pick (Gripper *g);
uint8_t Gripper_Place(Gripper *g);

/* ===========================================================================
 * White Pilot Lamp — PA1
 * =========================================================================*/
void PilotLamp_SetWhite(uint8_t on);   /* 1 = ON, 0 = OFF */

#endif