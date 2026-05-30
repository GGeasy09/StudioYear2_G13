/*
 * BaseSystem.h
 * Author: Ben
 */

#ifndef INC_BASESYSTEM_H_
#define INC_BASESYSTEM_H_

#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdbool.h> // ต้องมีสำหรับใช้งานตัวแปร bool

// --- ขนาดและข้อกำหนดพื้นฐาน ---
#define MODBUS_MESSAGEBUFFER_SIZE 300
#define BASE_SYSTEM_REG_COUNT 50

// --- โครงสร้าง 16-bit to 8-bit ---
typedef union {
    uint16_t U16;
    uint8_t U8[2];
} u16u8_t;

// --- State Machine ของ Modbus ---
typedef enum {
    Modbus_state_Init,
    Modbus_state_Idle,
    Modbus_state_Emission,
    Modbus_state_Reception,
    Modbus_state_ControlAndWaiting
} ModbusStateTypedef;

typedef enum {
    Modbus_function_Read_Holding_Register = 0x03,
    Modbus_function_Write_SingleRegister = 0x06
} ModbusFunctionCode;

typedef enum {
    Modbus_RecvFrame_Null = -2,
    Modbus_RecvFrame_FrameError = -1,
    Modbus_RecvFrame_Normal = 0,
    Modbus_RecvFrame_IllegalFunction = 1,
    Modbus_RecvFrame_IllegalDataAddress = 2,
    Modbus_RecvFrame_IllegalDataValue = 3,
    Modbus_RecvFrame_ServerDeviceFailure = 4
} modbusRecvFrameStatus;

typedef struct {
    uint8_t slaveAddress;
    u16u8_t *RegisterAddress;
    uint32_t RegisterSize;
    UART_HandleTypeDef* huart;
    TIM_HandleTypeDef* htim;
    uint8_t Flag_T15TimeOut;
    uint8_t Flag_T35TimeOut;
    modbusRecvFrameStatus RecvStatus;
    ModbusStateTypedef Mstatus;
    uint8_t Rxframe[MODBUS_MESSAGEBUFFER_SIZE];
    uint8_t Txframe[MODBUS_MESSAGEBUFFER_SIZE];
    uint8_t TxCount;
    struct {
        uint8_t MessageBufferRx[MODBUS_MESSAGEBUFFER_SIZE + 3];
        uint16_t RxTail;
        uint8_t MessageBufferTx[MODBUS_MESSAGEBUFFER_SIZE + 3];
        uint16_t TxTail;
    } modbusUartStructure;
} ModbusHandleTypedef;

// ==============================================================================
// ส่วนเพิ่มใหม่: ตัวแปลภาษา (Abstraction Layer) สำหรับให้เพื่อนใช้งาน
// ==============================================================================

typedef enum {
    CMD_MODE_IDLE      = 0,
    CMD_MODE_HOME      = 1,
    CMD_MODE_JOG       = 2,
    CMD_MODE_AUTO      = 4,
    CMD_MODE_SET_HOME  = 8,
    CMD_MODE_TEST      = 16
} BaseCmd_Mode_t;

typedef enum {
    CMD_GRIPPER_UP     = 0,
    CMD_GRIPPER_DOWN   = 1,
    CMD_GRIPPER_OPEN   = 2,
    CMD_GRIPPER_CLOSE  = 4
} BaseCmd_Gripper_t;

typedef enum {
    CMD_TEST_PRECISION = 0,
    CMD_TEST_PERFORM   = 1
} BaseCmd_TestType_t;

// โครงสร้างคำสั่งที่พร้อมใช้งาน
typedef struct {
    BaseCmd_Mode_t     Target_Mode;
    BaseCmd_Gripper_t  Gripper_Manual;
    uint8_t            Gripper_Seq;
    bool               Gripper_Auto_En;
    float              Jog_Degree;
    BaseCmd_TestType_t Test_Type;
    float              Test_Velocity;
    float              Test_Accel;
    float              Test_Init_Pos;
    float              Test_Final_Pos;
    float              Test_Repeat_Count;
    uint16_t           PickPlace_Pairs;
    int16_t            PickPlace_Sequence[10]; // เก็บ Sequence การทำงาน
    uint8_t            P2P_Unit;
    float              P2P_Target;
    bool               Soft_Stop_Req;
} BaseSystem_Commands_t;

// --- ประกาศตัวแปรส่วนกลาง ---
extern ModbusHandleTypedef hmodbus;
extern u16u8_t registerFrame[BASE_SYSTEM_REG_COUNT];
extern BaseSystem_Commands_t BaseCmd;
extern bool BaseSystem_IsConnected;

// --- ประกาศฟังก์ชันหลักของ Modbus ---
void Modbus_init(ModbusHandleTypedef* hmodbus, u16u8_t* RegisterStartAddress);
void Modbus_Protocal_Worker(void);

// --- ประกาศฟังก์ชัน API ให้เพื่อนใช้ส่งข้อมูลกลับไป UI ---
void BaseSystem_SendMotionStatus(float position, float velocity, float acceleration);
void BaseSystem_SendRobotTask(uint16_t task_bit, bool is_emergency);
void BaseSystem_SendSensorStatus(bool reed1_up, bool reed2_down, bool reed3_closed);

#endif /* INC_BASESYSTEM_H_ */
