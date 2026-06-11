/**
 * @file    gripper_can.c
 * @brief   CAN-bus gripper driver — Protocol Spec v1.0.1
 *
 * Usage:
 *   1. Call Gripper_CAN_Init() once after MX_FDCAN1_Init().
 *   2. Call Gripper_CAN_Tick(&robot_gripper) every main-loop iteration.
 *   3. Forward FDCAN RX interrupts:
 *        void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan) {
 *            FDCAN_RxHeaderTypeDef hdr;
 *            uint8_t data[8];
 *            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data);
 *            Gripper_CAN_ProcessRx(&robot_gripper, &hdr, data);
 *        }
 */

#include "gripper_can.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * Internal state
 * --------------------------------------------------------------------------*/
static uint8_t  s_relay_mask        = 0x00U;   /* shadow of current relay state */
static uint32_t s_last_hb_tick      = 0U;       /* timestamp of last heartbeat   */
static uint32_t s_last_opto_tick    = 0U;       /* timestamp of last opto poll   */

#define OPTO_POLL_PERIOD_MS  200U   /* how often to request opto state */

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------*/

/**
 * @brief  Send a CAN frame via FDCAN1 FIFO.
 * @return HAL_OK on success.
 */
static HAL_StatusTypeDef CAN_Transmit(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHdr;
    memset(&txHdr, 0, sizeof(txHdr));

    txHdr.Identifier          = id;
    txHdr.IdType              = FDCAN_STANDARD_ID;
    txHdr.TxFrameType         = FDCAN_DATA_FRAME;
    txHdr.DataLength          = (uint32_t)len << 16U;  /* FDCAN_DLC_BYTES_x */
    txHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHdr.BitRateSwitch       = FDCAN_BRS_OFF;
    txHdr.FDFormat            = FDCAN_CLASSIC_CAN;
    txHdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHdr.MessageMarker       = 0U;

    /* Map raw byte-count to HAL DLC constant */
    const uint32_t dlc_table[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    txHdr.DataLength = (len <= 8U) ? dlc_table[len] : FDCAN_DLC_BYTES_8;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHdr, (uint8_t *)data);
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/

void Gripper_CAN_Init(void)
{
    /* ---- Configure RX filters to accept frames from Node 0x10 ---- */
    FDCAN_FilterTypeDef filter;

    /* Filter 0: accept 0x110 (Opto broadcast) and 0x310 (Command Response) */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CAN_ID_CMD_RESP;   /* 0x310 — base ID  */
    filter.FilterID2    = 0x7F0U;            /* mask: top 7 bits, ignore low nibble */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Filter 1: accept 0x110 (periodic opto broadcast) */
    filter.FilterIndex  = 1U;
    filter.FilterID1    = CAN_ID_OPTO_BCAST; /* 0x110 */
    filter.FilterID2    = 0x7FFU;            /* exact match mask */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    /* Reject everything else */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT,
                                 FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    /* Enable RX FIFO 0 interrupt */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);

    /* Start peripheral */
    HAL_FDCAN_Start(&hfdcan1);

    /* Send initial heartbeat so node transitions to Operational immediately */
    Gripper_CAN_SendHeartbeat();
}

void Gripper_CAN_SendHeartbeat(void)
{
    uint8_t payload[1] = { CAN_MASTER_OPERATIONAL };
    CAN_Transmit(CAN_ID_MASTER_HB, payload, 1U);
    s_last_hb_tick = HAL_GetTick();
}

void Gripper_CAN_SendRelays(const Gripper *g)
{
    /* Rebuild relay mask from gripper logical state */
    s_relay_mask = 0x00U;

    if (g->grip_up == 1U)
        s_relay_mask |= RELAY_BIT_GRIPPER_UP;
    else
        s_relay_mask |= RELAY_BIT_GRIPPER_DOWN;

    if (g->grip_open == 1U)          /* grip_open=1 means OPEN */
        s_relay_mask |= RELAY_BIT_GRIPPER_OPEN;
    else
        s_relay_mask |= RELAY_BIT_GRIPPER_CLOSE;

    /* Write Relay command: [0x10][0x00][mask] */
    uint8_t payload[3] = {
        CAN_INSTR_WRITE_REQ,
        CAN_TARGET_RELAY_BANK,
        s_relay_mask
    };
    CAN_Transmit(CAN_ID_CMD_REQ, payload, 3U);
}

void Gripper_CAN_RequestOptoRead(Gripper *g)
{
    (void)g;  /* unused — kept for future use */
    uint8_t payload[2] = { CAN_INSTR_READ_REQ, CAN_TARGET_OPTO_BANK };
    CAN_Transmit(CAN_ID_CMD_REQ, payload, 2U);
    s_last_opto_tick = HAL_GetTick();
}

void Gripper_CAN_ProcessRx(Gripper *g,
                            const FDCAN_RxHeaderTypeDef *rxHeader,
                            const uint8_t *rxData)
{
    uint32_t id  = rxHeader->Identifier;
    uint32_t dlc = rxHeader->DataLength;  /* HAL DLC constant, >= FDCAN_DLC_BYTES_3 → len>=3 */

    /* Command Response (0x310) — Write Ack or Read Response */
    if (id == CAN_ID_CMD_RESP && dlc >= FDCAN_DLC_BYTES_3)
    {
        if (rxData[0] == CAN_INSTR_WRITE_ACK && rxData[1] == CAN_TARGET_RELAY_BANK)
        {
            /* Optional: verify ack matches our command */
            s_relay_mask = rxData[2];
        }
        else if (rxData[0] == CAN_INSTR_READ_RESP && rxData[1] == CAN_TARGET_OPTO_BANK)
        {
            uint8_t opto = rxData[2];
            g->is_up         = (opto & OPTO_BIT_GRIPPER_UP)   ? 1U : 0U;
            /* is_full_open / is_full_close mapped from open-close sensor bit */
            g->is_full_open  = (opto & OPTO_BIT_GRIPPER_OC)   ? 0U : 1U;
            g->is_full_close = (opto & OPTO_BIT_GRIPPER_OC)   ? 1U : 0U;
        }
    }

    /* Periodic Opto Broadcast (0x110) */
    if (id == CAN_ID_OPTO_BCAST && dlc >= FDCAN_DLC_BYTES_1)
    {
        uint8_t opto = rxData[0];
        g->is_up         = (opto & OPTO_BIT_GRIPPER_UP)   ? 1U : 0U;
        g->is_full_open  = (opto & OPTO_BIT_GRIPPER_OC)   ? 0U : 1U;
        g->is_full_close = (opto & OPTO_BIT_GRIPPER_OC)   ? 1U : 0U;
    }
}

void Gripper_CAN_Tick(Gripper *g)
{
    uint32_t now = HAL_GetTick();

    /* Send heartbeat every 500 ms */
    if ((now - s_last_hb_tick) >= GRIPPER_CAN_HB_PERIOD_MS)
        Gripper_CAN_SendHeartbeat();

    /* Poll opto inputs every 200 ms */
    if ((now - s_last_opto_tick) >= OPTO_POLL_PERIOD_MS)
        Gripper_CAN_RequestOptoRead(g);
}
