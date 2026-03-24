/**
 * @date 2025-11-6
*/

#ifndef __MODBUS_HOST_H_
#define __MODBUS_HOST_H_

#include "main.h"
#include "tool.h"

typedef struct {
    uint8_t slave_addr;
    uint8_t reg_count;
    uint16_t* registers;
    uint8_t reset_count;
} SlaveDevice;

/* 状态机状态定义 */
typedef enum {
    MODBUS_STATE_SENDING = 0,      // 发送请求
    MODBUS_STATE_PROCESSING,       // 处理响应
    MODBUS_STATE_ERROR             // 错误状态
} MODBUS_StateTypeDef;

// Modbus命令结构体
#define MAX_WRITE_REG_COUNT 40
typedef struct ModbusCommand {
    uint8_t slaveAddr;          // 从机地址
    uint8_t funcCode;           // 功能码
    uint16_t regAddr;           // 寄存器地址
    uint16_t regValue[MAX_WRITE_REG_COUNT];      // 寄存器值(用于写命令)
    uint16_t regCount;          // 寄存器数量
} ModbusCommand;

#define MODBUS_HOST_RX_BUF_SIZE 256
#define MODBUS_HOST_TX_BUF_SIZE 128
#define MODBUS_HOST_TX_RETRY    3
typedef struct modbus_host {
    UART_HandleTypeDef* p_uart;
    
    ring_buffer rx_buf;
    uint8_t cmd_buf[MODBUS_HOST_RX_BUF_SIZE];
    uint16_t cmd_size;
    ModbusCommand tran_cmd; // 发送的命令

    uint8_t tx_buf[MODBUS_HOST_TX_BUF_SIZE];
    uint8_t tx_size;

    uint8_t error_count; // 应答错误计数
    uint8_t miss_count;  // 无应答计数
    uint8_t retry;       // 重发次数

    MODBUS_StateTypeDef state;
}MODBUS_HostTypeDef;

static uint16_t Modbus_Calculate_CRC(uint8_t* data, uint16_t len);
static uint8_t Modbus_Transmit(MODBUS_HostTypeDef* pmb_host);
static uint8_t Modbus_Wait_RX_flag(uint16_t time);

uint8_t Modbus_Host_Init(MODBUS_HostTypeDef* pmb_host,
                         UART_HandleTypeDef* puart);
void Modbus_slave_init(SlaveDevice* slave_dev,
                       uint8_t slave_addr, uint16_t* p_reg, uint8_t reg_count);

uint8_t Modbus_Host_uart_callback(UART_HandleTypeDef* huart,
                                  uint16_t Size,
                                  MODBUS_HostTypeDef* pmb_host);

uint8_t Modbus_Parse_Data(MODBUS_HostTypeDef* pmb_host, SlaveDevice* p_slave);

uint8_t Modbus_Build_03(ModbusCommand* p_cmd, MODBUS_HostTypeDef* pmb_host);
uint8_t Modbus_Build_06(ModbusCommand* p_cmd, MODBUS_HostTypeDef* pmb_host);
uint8_t Modbus_Build_10(ModbusCommand* p_cmd, MODBUS_HostTypeDef* pmb_host);

uint8_t Modbus_Host_Send_Cmd(MODBUS_HostTypeDef* pmb_host, SlaveDevice* p_slave);

#endif //__MODBUS_HOST_H_
