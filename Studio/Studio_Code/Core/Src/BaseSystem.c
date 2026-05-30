/*
 * BaseSystem.c
 * Author: Ben
 */

#include "BaseSystem.h"

// --- ประกาศ Prototype ที่เรียกใช้ภายในไฟล์ ---
void Modbus_Emission(void);
void Modbus_frame_response(void);
void Decode_Incoming_Commands(void);
void BaseSystem_CheckHeartbeat(void);

ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[BASE_SYSTEM_REG_COUNT];

// ตัวแปรสำหรับใช้งานระดับ API
BaseSystem_Commands_t BaseCmd;
bool BaseSystem_IsConnected = false;

// ====================================================================
// ตาราง CRC16
// ====================================================================
static char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

unsigned short CRC16 (unsigned char *puchMsg, unsigned short usDataLen) {
    unsigned char uchCRCHi = 0xFF ;
    unsigned char uchCRCLo = 0xFF ;
    unsigned uIndex ;
    while (usDataLen--) {
        uIndex = uchCRCLo ^ *puchMsg++ ;
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
        uchCRCHi = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}

// ====================================================================
// การจัดการ Timeout และ Timer (Modbus Low-Level)
// ====================================================================
void modbus_1t5_Timeout(void) {
    hmodbus.Flag_T15TimeOut = 1;
    __HAL_TIM_SET_COUNTER(hmodbus.htim, 0);
    __HAL_TIM_ENABLE(hmodbus.htim);
}

void modbus_3t5_Timeout(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM16) {
        hmodbus.Flag_T35TimeOut = 1;
    }
}

void modbus_ErrorTimeout(UART_HandleTypeDef* huart) {
    if(HAL_UART_GetError(huart) == HAL_UART_ERROR_RTO) {
        modbus_1t5_Timeout();
    }
}

void Modbus_init(ModbusHandleTypedef* h, u16u8_t* RegisterStartAddress) {
    h->RegisterAddress = RegisterStartAddress;
    for(int i=0; i<h->RegisterSize; i++) {
        h->RegisterAddress[i].U16 = 0;
    }

    // 1. ใส่ค่า Heartbeat "YA" รอไว้เลย เพื่อให้ PC รู้ว่าหุ่นยนต์พร้อมคุย
    h->RegisterAddress[0].U16 = 22881;

    // 2. ตั้งค่าการจับเวลาว่างของสายสัญญาณ
    HAL_UART_ReceiverTimeout_Config(h->huart, 16);
    HAL_UART_EnableReceiverTimeout(h->huart);

    // *** 3. บังคับเปิด Interrupt เมื่อสายสัญญาณว่าง (สำคัญมาก!) ***
    __HAL_UART_ENABLE_IT(h->huart, UART_IT_RTO);

    // 4. เริ่มรอรับข้อมูลจาก DMA
    h->modbusUartStructure.RxTail = 0;
    HAL_UART_Receive_DMA(h->huart, h->modbusUartStructure.MessageBufferRx, MODBUS_MESSAGEBUFFER_SIZE);

    if(h->htim->State == HAL_TIM_STATE_READY) {
        HAL_TIM_Base_Start_IT(h->htim);
        HAL_TIM_OnePulse_Start_IT(h->htim, TIM_CHANNEL_1);
    }
    h->Mstatus = Modbus_state_Idle;
}
// ====================================================================
// ฟังก์ชันอ่าน/เขียนและแปลภาษา (API & Logic Layer)
// ====================================================================
void ModbusErrorReply(uint8_t Errorcode) {
    hmodbus.Txframe[0] = hmodbus.Rxframe[0] | 0x80;
    hmodbus.Txframe[1] = Errorcode;
    hmodbus.TxCount = 2;
}

void modbusRead1Register(void) {
    uint16_t startAddress = (hmodbus.Rxframe[1] << 8) | hmodbus.Rxframe[2];
    uint16_t numberOfDataToRead = (hmodbus.Rxframe[3] << 8) | hmodbus.Rxframe[4];

    if (numberOfDataToRead < 1 || numberOfDataToRead > 0x7D) {
        ModbusErrorReply(Modbus_RecvFrame_IllegalDataValue);
        return;
    }

    if (startAddress >= hmodbus.RegisterSize || (startAddress + numberOfDataToRead) > hmodbus.RegisterSize) {
        ModbusErrorReply(Modbus_RecvFrame_IllegalDataAddress);
        return;
    }

    hmodbus.Txframe[0] = Modbus_function_Read_Holding_Register;
    hmodbus.Txframe[1] = (numberOfDataToRead * 2) & 0xFF;

    for(int i=0; i<numberOfDataToRead; i++) {
        hmodbus.Txframe[2*i + 2] = hmodbus.RegisterAddress[startAddress + i].U8[1];
        hmodbus.Txframe[2*i + 3] = hmodbus.RegisterAddress[startAddress + i].U8[0];
    }
    hmodbus.TxCount = 2 + (2 * numberOfDataToRead);
}

void Decode_Incoming_Commands(void) {
    // 1. แปลงโหมด
    BaseCmd.Target_Mode = (BaseCmd_Mode_t)registerFrame[0x01].U16;

    // 2. แปลงคำสั่ง Gripper
    BaseCmd.Gripper_Manual  = (BaseCmd_Gripper_t)registerFrame[0x02].U16;
    BaseCmd.Gripper_Seq     = (uint8_t)registerFrame[0x03].U16;
    BaseCmd.Gripper_Auto_En = (registerFrame[0x04].U16 == 1);

    // 3. แปลงค่า Jog
    BaseCmd.Jog_Degree = (float)((int16_t)registerFrame[0x05].U16);

    // 4. แปลงคำสั่ง Test
    BaseCmd.Test_Type         = (BaseCmd_TestType_t)registerFrame[0x06].U16;
    BaseCmd.Test_Velocity     = (float)((int16_t)registerFrame[0x07].U16);
    BaseCmd.Test_Accel        = (float)((int16_t)registerFrame[0x08].U16);
    BaseCmd.Test_Init_Pos     = (float)((int16_t)registerFrame[0x09].U16);
    BaseCmd.Test_Final_Pos    = (float)((int16_t)registerFrame[0x10].U16);
    BaseCmd.Test_Repeat_Count = (float)((int16_t)registerFrame[0x11].U16);

    // 5. แปลงค่า Pick & Place
    BaseCmd.PickPlace_Pairs = registerFrame[0x22].U16;
    /* Positions 0-7: registers 0x12-0x19
     * Positions 8-9: registers 0x20-0x21 (BaseSystem skips 0x1A-0x1B) */
    for(int i = 0; i < 8; i++) {
        BaseCmd.PickPlace_Sequence[i] = (int16_t)registerFrame[0x12 + i].U16;
    }
    BaseCmd.PickPlace_Sequence[8] = (int16_t)registerFrame[0x20].U16;
    BaseCmd.PickPlace_Sequence[9] = (int16_t)registerFrame[0x21].U16;

    // 6. แปลง P2P และ Soft Stop
    BaseCmd.P2P_Unit      = (uint8_t)registerFrame[0x23].U16;
    BaseCmd.P2P_Target    = (float)((int16_t)registerFrame[0x24].U16);
    BaseCmd.Soft_Stop_Req = (registerFrame[0x25].U16 == 1);
}

void BaseSystem_CheckHeartbeat(void) {
    if(registerFrame[0x00].U16 == 18537) {
        BaseSystem_IsConnected = true;
        registerFrame[0x00].U16 = 22881;
    }
}

void modbusWritelRegister(void) {
    uint16_t startAddress = (hmodbus.Rxframe[1] << 8) | hmodbus.Rxframe[2];

    if(startAddress >= hmodbus.RegisterSize) {
        ModbusErrorReply(Modbus_RecvFrame_IllegalDataAddress);
        return;
    }

    registerFrame[startAddress].U8[1] = hmodbus.Rxframe[3];
    registerFrame[startAddress].U8[0] = hmodbus.Rxframe[4];

    /* Write-Single-Register echo = func + addrHi + addrLo + valHi + valLo = 5 bytes.
     * (slave address and CRC are appended later in Modbus_Emission.)
     * Was 6 — that copied a stale 6th byte, producing a 9-byte reply with a junk
     * byte before the CRC. In slow mode the idle gap flushed it; in AUTO bursts
     * there is no gap, so the stray byte desynced the master -> reconnect storm. */
    memcpy(hmodbus.Txframe, hmodbus.Rxframe, 5);
    hmodbus.TxCount = 5;

    Decode_Incoming_Commands();

    /* --- Set execution flags based on which register was just written ---
     * When one flag fires, all others are set to 99 for debugger visibility. */
    if (startAddress == 0x01 && BaseCmd.Target_Mode == CMD_MODE_SET_HOME)
    {
        BaseCmd.flag_sethome_execute = 1;
        BaseCmd.flag_jog_execute     = 99;
        BaseCmd.flag_p2p_execute     = 99;
        BaseCmd.flag_seq_execute     = 99;
        BaseCmd.flag_test_execute    = 99;
    }
    if (startAddress == 0x02)
        BaseCmd.flag_gripper_manual = 1;
    if (startAddress == 0x03)
        BaseCmd.flag_gripper_seq = 1;
    if (BaseCmd.Target_Mode == CMD_MODE_JOG && startAddress == 0x05)
    {
        BaseCmd.flag_jog_execute  = 1;
        BaseCmd.flag_p2p_execute  = 99;
        BaseCmd.flag_seq_execute  = 99;
        BaseCmd.flag_test_execute = 99;
    }
    if (BaseCmd.Target_Mode == CMD_MODE_AUTO && startAddress == 0x24)
    {
        BaseCmd.flag_p2p_execute  = 1;
        BaseCmd.flag_jog_execute  = 99;
        BaseCmd.flag_seq_execute  = 99;
        BaseCmd.flag_test_execute = 99;
    }
    if (BaseCmd.Target_Mode == CMD_MODE_AUTO && startAddress == 0x22)
    {
        BaseCmd.flag_seq_execute  = 1;
        BaseCmd.flag_jog_execute  = 99;
        BaseCmd.flag_p2p_execute  = 99;
        BaseCmd.flag_test_execute = 99;
    }
    /* PERFORM triggered by 0x08 (TestAccel), PRECISION triggered by 0x11 (TestRepeat) */
    if (BaseCmd.Target_Mode == CMD_MODE_TEST)
    {
        if ((BaseCmd.Test_Type == CMD_TEST_PERFORM  && startAddress == 0x08) ||
            (BaseCmd.Test_Type == CMD_TEST_PRECISION && startAddress == 0x11))
        {
            BaseCmd.flag_test_execute = 1;
            BaseCmd.flag_jog_execute  = 99;
            BaseCmd.flag_p2p_execute  = 99;
            BaseCmd.flag_seq_execute  = 99;
        }
    }

    if(startAddress == 0x00) {
        BaseSystem_CheckHeartbeat();
    }
}

void Modbus_frame_response(void) {
    switch (hmodbus.Rxframe[0]) {
        case Modbus_function_Write_SingleRegister:
            modbusWritelRegister();
            break;
        case Modbus_function_Read_Holding_Register:
            modbusRead1Register();
            break;
        default:
            ModbusErrorReply(Modbus_RecvFrame_IllegalFunction);
            break;
    }
}

void Modbus_Emission(void) {
    if(hmodbus.huart->gState == HAL_UART_STATE_READY) {
        hmodbus.modbusUartStructure.MessageBufferTx[0] = hmodbus.slaveAddress;
        memcpy(hmodbus.modbusUartStructure.MessageBufferTx + 1, hmodbus.Txframe, hmodbus.TxCount);
        hmodbus.modbusUartStructure.TxTail = hmodbus.TxCount + 3;

        u16u8_t CalculateCRC;
        CalculateCRC.U16 = CRC16(hmodbus.modbusUartStructure.MessageBufferTx, hmodbus.modbusUartStructure.TxTail - 2);

        hmodbus.modbusUartStructure.MessageBufferTx[hmodbus.modbusUartStructure.TxTail - 2] = CalculateCRC.U8[0];
        hmodbus.modbusUartStructure.MessageBufferTx[hmodbus.modbusUartStructure.TxTail - 1] = CalculateCRC.U8[1];

        if(hmodbus.huart->gState == HAL_UART_STATE_READY) {
            HAL_UART_Transmit_DMA(hmodbus.huart, hmodbus.modbusUartStructure.MessageBufferTx, hmodbus.modbusUartStructure.TxTail);
        }

        hmodbus.TxCount = 0;
        hmodbus.Mstatus = Modbus_state_Idle;
    }
}

// ====================================================================
// State Machine หลัก (ฉบับแก้ไขบัค DMA ค้าง)
// ====================================================================
void Modbus_Protocal_Worker(void) {
    switch(hmodbus.Mstatus) {

        case Modbus_state_Init:
            hmodbus.Mstatus = Modbus_state_Idle;
            break;

        case Modbus_state_Idle:
            if(hmodbus.TxCount > 0) {
                hmodbus.Mstatus = Modbus_state_Emission;
                Modbus_Emission();
            }
            // ถ้า Timeout เด้ง แปลว่ารับข้อมูลครบ 8 ไบต์แล้ว ให้กระโดดไปจัดการข้อมูล
            else if(hmodbus.Flag_T15TimeOut) {
                hmodbus.Mstatus = Modbus_state_Reception;
            }
            // ถ้า UART ว่าง ให้เปิด DMA รอรับข้อมูลชุดใหม่
            else if(hmodbus.huart->RxState == HAL_UART_STATE_READY) {
                hmodbus.modbusUartStructure.RxTail = 0;
                HAL_UART_Receive_DMA(hmodbus.huart, &hmodbus.modbusUartStructure.MessageBufferRx[0], MODBUS_MESSAGEBUFFER_SIZE);
            }

            break;

        case Modbus_state_Reception:
            if(hmodbus.Flag_T15TimeOut) {
                hmodbus.Flag_T15TimeOut = 0;
                hmodbus.RecvStatus = Modbus_RecvFrame_Null;

                // 1. คำนวณว่ารับข้อมูลมาได้กี่ไบต์ (เช่น 8 ไบต์)
                hmodbus.modbusUartStructure.RxTail = hmodbus.huart->RxXferSize - __HAL_DMA_GET_COUNTER(hmodbus.huart->hdmarx);

                // 2. ยกเลิก DMA เดิมที่ค้างอยู่
                HAL_UART_AbortReceive(hmodbus.huart);

                /* 3. Snapshot the frame into ParseBuffer, then RE-ARM RX DMA
                 *    immediately. Previously RX stayed off all the way through
                 *    parsing + the (up to ~5 ms) reply transmit, so any frame
                 *    the master sent in that window was lost -> timeouts ->
                 *    disconnect under AUTO bursts. Now RX is deaf for only a
                 *    few microseconds. ParseBuffer is separate from
                 *    MessageBufferRx so an incoming frame can't clobber the
                 *    one we are still parsing. */
                hmodbus.ParseLen = hmodbus.modbusUartStructure.RxTail;
                if (hmodbus.ParseLen > (MODBUS_MESSAGEBUFFER_SIZE + 3))
                    hmodbus.ParseLen = (MODBUS_MESSAGEBUFFER_SIZE + 3);
                memcpy(hmodbus.ParseBuffer,
                       hmodbus.modbusUartStructure.MessageBufferRx,
                       hmodbus.ParseLen);

                hmodbus.modbusUartStructure.RxTail = 0;
                HAL_UART_Receive_DMA(hmodbus.huart,
                                     hmodbus.modbusUartStructure.MessageBufferRx,
                                     MODBUS_MESSAGEBUFFER_SIZE);

                hmodbus.Mstatus = Modbus_state_ControlAndWaiting;
            }
            break;

        case Modbus_state_ControlAndWaiting:
            if(hmodbus.RecvStatus == Modbus_RecvFrame_Null) {
                hmodbus.RecvStatus = Modbus_RecvFrame_Normal;

                // ป้องกันระบบค้าง หากมีข้อมูลขยะส่งมาสั้นกว่า 4 ไบต์
                if (hmodbus.ParseLen < 4) {
                    hmodbus.RecvStatus = Modbus_RecvFrame_FrameError;
                    hmodbus.frame_error_count++;
                    hmodbus.Mstatus = Modbus_state_Idle;
                    break;
                }

                // ตรวจสอบ CRC  (parse the snapshot, not the live RX buffer)
                u16u8_t CalculateCRC;
                CalculateCRC.U16 = CRC16(hmodbus.ParseBuffer,
                                         hmodbus.ParseLen - 2);

                if(!(CalculateCRC.U8[0] == hmodbus.ParseBuffer[hmodbus.ParseLen - 2] &&
                     CalculateCRC.U8[1] == hmodbus.ParseBuffer[hmodbus.ParseLen - 1])) {
                    hmodbus.RecvStatus = Modbus_RecvFrame_FrameError;
                    hmodbus.crc_error_count++;
                    hmodbus.Mstatus = Modbus_state_Idle;
                    break;
                }

                // ตรวจสอบ Slave Address
                if(hmodbus.ParseBuffer[0] != hmodbus.slaveAddress) {
                    hmodbus.Mstatus = Modbus_state_Idle;
                    break;
                }

                // ก๊อปปี้ข้อมูลไปใช้งาน
                memcpy(hmodbus.Rxframe,
                       &hmodbus.ParseBuffer[1],
                       hmodbus.ParseLen - 3);

                hmodbus.frame_ok_count++;
                Modbus_frame_response(); // โค้ดจะวิ่งไปแปลภาษาให้เพื่อนตรงนี้แหละครับ!
            }

            if(hmodbus.Flag_T35TimeOut) {
                hmodbus.Flag_T35TimeOut = 0;
                hmodbus.Mstatus = Modbus_state_Idle;
            }
            break;

        case Modbus_state_Emission:
            if(hmodbus.huart->gState == HAL_UART_STATE_READY) {
                hmodbus.TxCount = 0;
                hmodbus.Mstatus = Modbus_state_Idle;
            }
            break;

        default:
            hmodbus.Mstatus = Modbus_state_Idle;
            break;
    }
}

// ====================================================================
// ฟังก์ชันส่งข้อมูลให้เพื่อนเรียกใช้ (API for Motor Control)
// ====================================================================
void BaseSystem_SendMotionStatus(float position, float velocity, float acceleration) {
    registerFrame[0x28].U16 = (uint16_t)((int16_t)(position * 10.0f));
    registerFrame[0x29].U16 = (uint16_t)((int16_t)(velocity * 10.0f));
    registerFrame[0x30].U16 = (uint16_t)((int16_t)(acceleration * 10.0f));
}

void BaseSystem_SendRobotTask(uint16_t task_bit, bool is_emergency) {
    registerFrame[0x27].U16 = task_bit;
    if(is_emergency) registerFrame[0x31].U16 = 0x0001;
    else             registerFrame[0x31].U16 = 0x0000;
}

void BaseSystem_SendSensorStatus(bool reed1_up, bool reed2_down, bool reed3_closed) {
    uint16_t sensor_status = 0;
    if(reed1_up)     sensor_status |= 0x0001;
    if(reed2_down)   sensor_status |= 0x0002;
    if(reed3_closed) sensor_status |= 0x0004;

    registerFrame[0x26].U16 = sensor_status;
}

// ====================================================================
// การจัดการ Interrupt แบบมาตรฐานของ STM32 (Standard HAL Callbacks)
// ====================================================================

// ฟังก์ชันนี้จะทำงานอัตโนมัติเมื่อ UART รับข้อมูลครบและสายว่าง (Receiver Timeout)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == hmodbus.huart->Instance) {
        if(HAL_UART_GetError(huart) == HAL_UART_ERROR_RTO) {
            modbus_1t5_Timeout(); // เริ่มเปิด Timer จับเวลาต่อ
        }
    }
}

// ฟังก์ชันนี้จะทำงานอัตโนมัติเมื่อ Timer 16 นับเวลาครบ 3.5t
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == hmodbus.htim->Instance) {
        modbus_3t5_Timeout(htim); // ส่งสัญญาณให้ State Machine เริ่มประมวลผลข้อมูล
    }
}