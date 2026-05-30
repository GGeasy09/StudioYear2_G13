#include "elec_cabient.h"

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
    joy->soft_stop       = !joy->raw_btn_soft_stop; 
    
    /* 3. Handle your hardware quirk: "Right white button also triggers Left white button" */
    if (joy->btn_white_right) {
        joy->btn_white_left = 1; 
    }
}

void SYSTEM_STATE_PilotRamp_SetExclusive(Pilot_ramp *ramp, uint8_t activate_relay_num)
{
    /* 1. Update internal structure states cleanly */
    ramp->relay1 = (activate_relay_num == 1) ? 1 : 0;
    ramp->relay2 = (activate_relay_num == 2) ? 1 : 0;

    /* 2. Write states directly to the new STM32 Hardware GPIO Pins */
    
    // Relay 1 / Light 1 -> PC8
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, ramp->relay1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // Relay 2 / Light 2 -> PC9
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, ramp->relay2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}