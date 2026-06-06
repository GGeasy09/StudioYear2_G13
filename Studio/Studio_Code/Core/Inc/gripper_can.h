/**
 * @file    gripper_can.h
 * @brief   CAN-bus gripper driver — Protocol Spec v1.0.1
 *
 * Relay Bank 0 mapping (per spec page 9):
 *   Relay 0 = Gripper Up Solenoid
 *   Relay 1 = Gripper Down Solenoid
 *   Relay 2 = Gripper Close Solenoid
 *   Relay 3 = Gripper Open Solenoid
 *
 * Opto Bank 0 sensor feedback (per spec page 9):
 *   Opto 4  = Gripper Up Sensor
 *   Opto 5  = Gripper Down Sensor
 *   Opto 6  = Gripper Open/Close Sensor
 */

#ifndef GRIPPER_CAN_H
#define GRIPPER_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include "elec_cabient.h"   /* Gripper struct */

/* ---------------------------------------------------------------------------
 * Protocol constants
 * --------------------------------------------------------------------------*/
#define GRIPPER_NODE_ID          0x10U

/** Build an 11-bit CAN ID from a 3-bit function code and 8-bit node ID. */
#define MAKE_CAN_ID(func, node)  (((uint32_t)((func) & 0x07U) << 8) | ((node) & 0xFFU))

/* CAN IDs used by this driver */
#define CAN_ID_CMD_REQ           MAKE_CAN_ID(0x02U, GRIPPER_NODE_ID)  /* 0x210 Master→Node  */
#define CAN_ID_CMD_RESP          MAKE_CAN_ID(0x03U, GRIPPER_NODE_ID)  /* 0x310 Node→Master  */
#define CAN_ID_OPTO_BCAST        MAKE_CAN_ID(0x01U, GRIPPER_NODE_ID)  /* 0x110 periodic     */
#define CAN_ID_MASTER_HB         0x600U                                /* heartbeat broadcast*/

/* Instruction bytes (spec §4.2 / §4.3) */
#define CAN_INSTR_WRITE_REQ      0x10U
#define CAN_INSTR_WRITE_ACK      0x11U
#define CAN_INSTR_READ_REQ       0x20U
#define CAN_INSTR_READ_RESP      0x21U

/* Target banks (spec §4.1) */
#define CAN_TARGET_RELAY_BANK    0x00U
#define CAN_TARGET_OPTO_BANK     0x10U

/* Master state bytes (spec §3.1) */
#define CAN_MASTER_OPERATIONAL   0x05U
#define CAN_MASTER_STOPPED       0x00U

/* ---------------------------------------------------------------------------
 * Relay bit-mask helpers (Byte 2 of Write-Relay command)
 * --------------------------------------------------------------------------*/
#define RELAY_BIT_GRIPPER_UP     (1U << 0)   /* Relay 0 */
#define RELAY_BIT_GRIPPER_DOWN   (1U << 1)   /* Relay 1 */
#define RELAY_BIT_GRIPPER_CLOSE  (1U << 2)   /* Relay 2 */
#define RELAY_BIT_GRIPPER_OPEN   (1U << 3)   /* Relay 3 */

/* ---------------------------------------------------------------------------
 * Opto bit-mask helpers (Byte 2 of Read-Response / 0x110 broadcast)
 * --------------------------------------------------------------------------*/
#define OPTO_BIT_GRIPPER_UP      (1U << 4)   /* Opto 4 — Up sensor    */
#define OPTO_BIT_GRIPPER_DOWN    (1U << 5)   /* Opto 5 — Down sensor  */
#define OPTO_BIT_GRIPPER_OC      (1U << 6)   /* Opto 6 — Open/Close   */

/* ---------------------------------------------------------------------------
 * Heartbeat period
 * --------------------------------------------------------------------------*/
#define GRIPPER_CAN_HB_PERIOD_MS  500U   /* spec §3.1: heartbeat every 500 ms */

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/

/**
 * @brief  One-time setup: configure FDCAN filters to accept responses from
 *         Node 0x10 (IDs 0x110, 0x310) and start the peripheral.
 *         Call once after MX_FDCAN1_Init().
 */
void Gripper_CAN_Init(void);

/**
 * @brief  Send the Master Heartbeat (CAN ID 0x600).
 *         Call every GRIPPER_CAN_HB_PERIOD_MS (500 ms) to keep the node alive.
 */
void Gripper_CAN_SendHeartbeat(void);

/**
 * @brief  Write relay state derived from Gripper struct fields (grip_open, grip_up).
 *         Builds the 4-bit relay mask and sends CAN ID 0x210.
 * @param  g  Pointer to Gripper struct (grip_open / grip_up must be set first).
 */
void Gripper_CAN_SendRelays(const Gripper *g);

/**
 * @brief  Send a Read-Opto request (CAN ID 0x210, Read Request).
 *         The node replies with 0x310; call Gripper_CAN_ProcessRx() in the
 *         FDCAN RX callback to update the Gripper sensor fields.
 * @param  g  Pointer to Gripper struct (unused now, future-proofing).
 */
void Gripper_CAN_RequestOptoRead(Gripper *g);

/**
 * @brief  Parse an incoming CAN frame and update Gripper sensor fields.
 *         Call from HAL_FDCAN_RxFifo0MsgPendingCallback() or polling loop.
 * @param  g        Pointer to Gripper struct to update.
 * @param  rxHeader Pointer to the received FDCAN header.
 * @param  rxData   Pointer to the received data bytes (up to 8).
 */
void Gripper_CAN_ProcessRx(Gripper *g,
                            const FDCAN_RxHeaderTypeDef *rxHeader,
                            const uint8_t *rxData);

/**
 * @brief  Convenience wrapper: tick function to call from main loop.
 *         Sends heartbeat when due, optionally polls opto state.
 * @param  g  Pointer to Gripper struct.
 */
void Gripper_CAN_Tick(Gripper *g);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_CAN_H */
