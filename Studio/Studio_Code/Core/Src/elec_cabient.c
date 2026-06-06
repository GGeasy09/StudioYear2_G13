#include "elec_cabient.h"
#if (GRIPPER_USE_CAN == 1)
#include "gripper_can.h"
#endif

void SYSTEM_STATE_Joystick_Update(Joystick *joy)
{
    /* 1. Read Raw Hardware Inputs (Pull-up: Released = 1, Pressed = 0) */
    joy->raw_mode            = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    joy->raw_btn_black       = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    joy->raw_btn_red         = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    joy->raw_btn_white_left  = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    joy->raw_btn_white_right = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    joy->raw_btn_blue        = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    joy->raw_btn_yellow      = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    joy->raw_btn_soft_stop   = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);

    /* 2. Convert to Active-High Logical States (Pressed = 1, Released = 0) */
    joy->mode            = !joy->raw_mode;
    joy->btn_black       = !joy->raw_btn_black;
    joy->btn_red         = !joy->raw_btn_red;
    joy->btn_white_left  = !joy->raw_btn_white_left;
    joy->btn_white_right = !joy->raw_btn_white_right;
    joy->btn_blue        = !joy->raw_btn_blue;
    joy->btn_yellow      = !joy->raw_btn_yellow;
    joy->soft_stop       = !joy->raw_btn_soft_stop;   /* pull-up: 0=pressed/triggered, 1=released/safe */
    
    /* 3. Handle your hardware quirk: "Right white button also triggers Left white button" */
    if (joy->btn_white_right) {
        joy->btn_white_left = 1; 
    }
}

/* relay1: 1 = Power ON / Emer OFF,  0 = Emer ON / Power OFF */
void PilotRamp_SetPower(Pilot_ramp *ramp, uint8_t on)
{
    ramp->relay1 = on;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, on);
}

/* relay2: 1 = Manual ON / Auto OFF,  0 = Auto ON / Manual OFF */
void PilotRamp_SetAuto(Pilot_ramp *ramp, uint8_t on)
{
    ramp->relay2 = on;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, on);
}

/* ===========================================================================
 * Gripper
 * =========================================================================*/

/* ===========================================================================
 * Gripper — transport-swappable implementation
 *
 * Set  GRIPPER_USE_CAN 0  in elec_cabient.h (or via project-level define)
 * to use direct GPIO (original behaviour).
 * Set  GRIPPER_USE_CAN 1  to route all output commands through CAN bus
 * (Protocol Spec v1.0.1, Node 0x10, Relay Bank 0).
 * =========================================================================*/

/* PC4 Relay3: cmd 0=open, 1=close */
void Gripper_SetOpen(Gripper *g, uint8_t cmd)
{
    g->grip_open = cmd;
#if (GRIPPER_USE_CAN == 0)
    /* --- GPIO path --- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, cmd ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    /* --- CAN path --- */
    Gripper_CAN_SendRelays(g);   /* sends full relay mask including grip_up */
#endif
}

/* PC5 Relay4: cmd 0=down, 1=up */
void Gripper_SetUp(Gripper *g, uint8_t cmd)
{
    g->grip_up = cmd;
#if (GRIPPER_USE_CAN == 0)
    /* --- GPIO path --- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, cmd ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    /* --- CAN path --- */
    Gripper_CAN_SendRelays(g);   /* sends full relay mask including grip_open */
#endif
}

/* Read all sensor states.
 * GPIO:  direct pin read (pull-up, 1 = sensor active).
 * CAN:   struct fields are updated asynchronously by Gripper_CAN_ProcessRx();
 *        this function is a no-op in CAN mode — call Gripper_CAN_Tick() instead. */
void Gripper_ReadState(Gripper *g)
{
#if (GRIPPER_USE_CAN == 0)
    /* --- GPIO path --- */
    g->is_full_open  = (uint8_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);   /* PC6:  1=fully open  */
    g->is_full_close = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);  /* PB12: 1=fully closed */
    g->is_up         = (uint8_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);   /* PC2:  1=up, 0=down  */
#else
    /* --- CAN path ---
     * Sensor state is refreshed by Gripper_CAN_ProcessRx() whenever the node
     * sends a Command Response (0x310) or periodic Opto Broadcast (0x110).
     * Nothing to do here — fields in *g are already up-to-date. */
    (void)g;
#endif
}   

/* ===========================================================================
 * White Pilot Lamp — PA1: 1 = ON, 0 = OFF
 * =========================================================================*/
void PilotLamp_SetWhite(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===========================================================================
 * Gripper_Pick  — non-blocking state machine
 * Call every main loop tick. Returns 1 when complete, 0 while running.
 *
 * Phase 0: lower arm
 * Phase 1: wait arm down → close gripper
 * Phase 2: wait gripper closed → raise arm
 * Phase 3: wait arm up → done
 * =========================================================================*/
uint8_t Gripper_Pick(Gripper *g)
{
    uint32_t delay = (g->action_delay_ms > 0) ? g->action_delay_ms : GRIPPER_PHASE_DELAY_MS;

    switch (g->action_phase)
    {
        case 0:
            Gripper_SetUp(g, 0);        /* lower arm */
            g->phase_start  = HAL_GetTick();
            g->action_phase = 1;
            break;

        case 1:
            if (HAL_GetTick() - g->phase_start >= delay)
            {
                Gripper_SetOpen(g, 1);  /* close gripper */
                g->phase_start  = HAL_GetTick();
                g->action_phase = 2;
            }
            break;

        case 2:
            if (HAL_GetTick() - g->phase_start >= delay)
            {
                Gripper_SetUp(g, 1);    /* raise arm */
                g->phase_start  = HAL_GetTick();
                g->action_phase = 3;
            }
            break;

        case 3:
            if (HAL_GetTick() - g->phase_start >= delay)
            {
                g->action_phase = 0;
                return 1;              /* pick complete */
            }
            break;
    }
    return 0;
}

/* ===========================================================================
 * Gripper_Place  — non-blocking state machine
 * Call every main loop tick. Returns 1 when complete, 0 while running.
 *
 * Phase 0: lower arm
 * Phase 1: wait arm down → open gripper
 * Phase 2: wait gripper open → raise arm
 * Phase 3: wait arm up → done
 * =========================================================================*/
uint8_t Gripper_Place(Gripper *g)
{
    uint32_t delay = (g->action_delay_ms > 0) ? g->action_delay_ms : GRIPPER_PHASE_DELAY_MS;

    switch (g->action_phase)
    {
        case 0:
            Gripper_SetUp(g, 0);        /* lower arm */
            g->phase_start  = HAL_GetTick();
            g->action_phase = 1;
            break;

        case 1:
            if (HAL_GetTick() - g->phase_start >= delay)
            {
                Gripper_SetOpen(g, 0);  /* open gripper */
                g->phase_start  = HAL_GetTick();
                g->action_phase = 2;
            }
            break;

        case 2:
            if (HAL_GetTick() - g->phase_start >= delay)
            {
                Gripper_SetUp(g, 1);    /* raise arm */
                g->phase_start  = HAL_GetTick();
                g->action_phase = 3;
            }
            break;

        case 3:
            if (HAL_GetTick() - g->phase_start >= delay)
            {
                g->action_phase = 0;
                return 1;              /* place complete */
            }
            break;
    }
    return 0;
}