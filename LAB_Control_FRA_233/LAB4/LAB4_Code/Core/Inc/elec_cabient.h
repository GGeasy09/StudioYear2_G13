#ifndef JOY_STICK_H
#define JOY_STICK_H
#include <stdint.h>
#include "arm_math.h" /* CMSIS-DSP Library */
#include "gpio.h"
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
    uint8_t relay1;   /* PC8: Relay 1 / Light 1 logical state (0 = OFF, 1 = ON) */
    uint8_t relay2;   /* PC9: Relay 2 / Light 2 logical state (0 = OFF, 1 = ON) */
} Pilot_ramp;
void SYSTEM_STATE_PilotRamp_SetExclusive(Pilot_ramp *ramp, uint8_t activate_relay_num);

#endif