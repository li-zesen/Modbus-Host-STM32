/**
 * @date 2025-11-6
*/

#include "modbus_host.h"
#include "tool.h"
#include "FreeRTOS.h"
#include "task.h"

extern CRC_HandleTypeDef hcrc;

/**
 * @brief 计算Modbus CRC16校验码（使用CRC外设HAL库）
 * @param data 数据的指针
 * @param len 数据长度
 * @return CRC值，uint16_t
*/
static uint16_t Modbus_Calculate_CRC(uint8_t* data, uint16_t len)
{
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, len);
    return (uint16_t)crc;
}

static uint8_t Modbus_Transmit(MODBUS_HostTypeDef* pmb_host)
{
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(pmb_host->p_uart,
                                                  pmb_host->tx_buf,
                                                  pmb_host->tx_size);
    return (uint8_t)ret;
}

/**
 * @brief 等待应答，0-无应答
*/
static uint8_t Modbus_Wait_RX_flag(uint16_t time)
{
    uint8_t ret = 0;
    ret = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(time));
    return ret;
}

/**
 * @brief Modbus构建03命令帧，需要slaveAddr、regAddr、regCount
 * @param p_cmd 要发送的命令结构体指针
 * @param pmb_host 发送命令的主机
 * @return 0-成功，1-寄存器数量错误，2-数据超过缓冲区长度
*/
uint8_t Modbus_Build_03(ModbusCommand* p_cmd, MODBUS_HostTypeDef* pmb_host)
{
    if (p_cmd->regCount == 0 || p_cmd->regCount > 125) return 1;  // 03命令最大125个寄存器

    uint8_t count = 0;
    pmb_host->tx_size = 0;

    pmb_host->tx_buf[count++] = p_cmd->slaveAddr;
    pmb_host->tx_buf[count++] = 0x03;
    pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regAddr);  // 寄存器地址高字节
    pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regAddr);   // 寄存器地址低字节
    pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regCount); // 数量高字节
    pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regCount);  // 数量低字节
    
    // 计算并添加CRC
    uint16_t crc = Modbus_Calculate_CRC(pmb_host->tx_buf, 6);
    pmb_host->tx_buf[count++] = crc & 0xFF;
    pmb_host->tx_buf[count++] = (crc >> 8) & 0xFF;

    pmb_host->tx_size = count;
    return 0;
}

/**
 * @brief Modbus构建06命令帧，写单个保持寄存器，需要slaveAddr、regAddr、regValue[0]
 * @param p_cmd 要发送的命令结构体指针
 * @param pmb_host 发送命令的主机
 * @return 0-成功
*/
uint8_t Modbus_Build_06(ModbusCommand* p_cmd, MODBUS_HostTypeDef* pmb_host)
{
    uint8_t count = 0;
    pmb_host->tx_size = 0;

    pmb_host->tx_buf[count++] = p_cmd->slaveAddr;
    pmb_host->tx_buf[count++] = 0x06;
    pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regAddr);  // 寄存器地址高字节
    pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regAddr);   // 寄存器地址低字节
    pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regValue[0]); // 数据高字节
    pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regValue[0]);  // 数据低字节
    
    // 计算并添加CRC
    uint16_t crc = Modbus_Calculate_CRC(pmb_host->tx_buf, 6);
    pmb_host->tx_buf[count++] = crc & 0xFF;
    pmb_host->tx_buf[count++] = (crc >> 8) & 0xFF;

    pmb_host->tx_size = count;
    return 0;
}

/**
 * @brief Modbus构建0x10命令帧，写多个保持寄存器，需要slaveAddr、regAddr、regValue[]
 * @param p_cmd 要发送的命令结构体指针
 * @param pmb_host 发送命令的主机
 * @return 0-成功，1-寄存器数量错误，2-数据超过缓冲区长度
*/
uint8_t Modbus_Build_10(ModbusCommand* p_cmd, MODBUS_HostTypeDef* pmb_host)
{
    uint8_t i = 0;
    uint8_t count = 0;
    pmb_host->tx_size = 0;
    if (p_cmd->regCount == 0 || p_cmd->regCount > MAX_WRITE_REG_COUNT) {
        return 1;
    }

    pmb_host->tx_buf[count++] = p_cmd->slaveAddr;
    pmb_host->tx_buf[count++] = 0x10;
    pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regAddr);  // 寄存器地址高字节
    pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regAddr);   // 寄存器地址低字节
    pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regCount); // 寄存器数量高字节
    pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regCount);  // 寄存器数量低字节
    pmb_host->tx_buf[count++] = p_cmd->regCount * 2;           // 数据字节数

    for (i = 0; i < p_cmd->regCount; i++) {
        if (count > MODBUS_HOST_TX_BUF_SIZE - 3) {
            return 2;		/* 数据超过缓冲区长度，直接丢弃不发送 */
        }
        pmb_host->tx_buf[count++] = GET_u16_HIGH(p_cmd->regValue[i]);
        pmb_host->tx_buf[count++] = GET_u16_LOW(p_cmd->regValue[i]);
    }
    
    // 计算并添加CRC
    uint16_t crc = Modbus_Calculate_CRC(pmb_host->tx_buf, count);
    pmb_host->tx_buf[count++] = crc & 0xFF;
    pmb_host->tx_buf[count++] = (crc >> 8) & 0xFF;

    pmb_host->tx_size = count;
    return 0;
}

/**
 * @brief 初始化从机设备结构体的值，从机地址，寄存器地址数组，寄存器数量
 * @param slave_dev 指向从机设备结构体的指针
 * @param slave_addr 从机地址
 * @param p_reg 指向寄存器数组的指针，数量必须==reg_count
 * @param reg_count 寄存器数量，不能超过MAX_REG_COUNT_PER_SLAVE
*/
void Modbus_slave_init(SlaveDevice* slave_dev,
                uint8_t slave_addr, uint16_t* p_reg, uint8_t reg_count)
{
    slave_dev->reg_count = reg_count;
    slave_dev->slave_addr = slave_addr;
    slave_dev->registers = p_reg;
    slave_dev->reset_count = 0;
}

uint8_t Modbus_Host_Init(MODBUS_HostTypeDef* pmb_host, UART_HandleTypeDef* puart)
{
    if (pmb_host == NULL || puart == NULL) {
        return 1;
    }
    pmb_host->p_uart = puart;
    pmb_host->rx_buf.write_pos = 0;
    pmb_host->rx_buf.read_pos = 0;
    pmb_host->miss_count = 0;
    pmb_host->error_count = 0;
    pmb_host->retry = MODBUS_HOST_TX_RETRY;

    HAL_UARTEx_ReceiveToIdle_DMA(pmb_host->p_uart,
                                 pmb_host->rx_buf.buffer, RING_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(pmb_host->p_uart->hdmarx, DMA_IT_HT);// 关闭DMA传输半完成中断
    //__HAL_DMA_DISABLE_IT(pmb_host->p_uart->hdmarx, DMA_IT_TC);// 关闭DMA传输完成中断
    return 0;
}

/**
 * @brief 串口接收完成回调函数，用的是空闲中断，将环形缓冲区内的数据复制到cmd_buf
*/
uint8_t Modbus_Host_uart_callback(UART_HandleTypeDef* huart, uint16_t Size,
                                  MODBUS_HostTypeDef* pmb_host)
{
    if(huart == pmb_host->p_uart) {
        pmb_host->rx_buf.write_pos = Size;
        pmb_host->cmd_size = ring_buffer_copy_all(&pmb_host->rx_buf,
                                                  pmb_host->cmd_buf,
                                                  MODBUS_HOST_RX_BUF_SIZE);
        return 0;
    }
    return 1;
}

/**
 * @brief 处理接收到的，从缓冲区内提取出来的Modbus命令
*/
uint8_t Modbus_Parse_Data(MODBUS_HostTypeDef* pmb_host, SlaveDevice* p_slave)
{
    uint8_t* pbuf = pmb_host->cmd_buf;
    uint16_t len = pmb_host->cmd_size;
    ModbusCommand* ptran = &pmb_host->tran_cmd;
    uint16_t i = 0;
    uint16_t reg_addr = 0;;
    uint16_t reg_value = 0;
    uint16_t reg_count = 0;

    // 最小长度
    if (len < 5) {
        return 1; // 错误：长度不足
    }
    // 检查从机地址和功能码
    if (pbuf[0] != ptran->slaveAddr || pbuf[1] != ptran->funcCode) {
        return 2;
    }
    // 校验CRC
    uint16_t received_crc = u8_to_u16(pbuf[len - 1], pbuf[len - 2]);
    uint16_t calculated_crc = Modbus_Calculate_CRC(pbuf, len - 2);
    
    if (received_crc != calculated_crc) {
        return 3;
    }

    // 根据功能码处理响应数据
    if (pbuf[1] == 0x03) {
        if (pbuf[2] != ptran->regCount * 2) {// 数据长度 字节数
            return 31;
        }

        if (pbuf[0] != p_slave->slave_addr) { // 检查赋值的从机地址
            return 32;
        }
        
        reg_addr = ptran->regAddr;
        // 检查寄存器数量是否超出范围
        if ((reg_addr + ptran->regCount) > p_slave->reg_count) {
            return 33;
        }
        for (i = 0;i < ptran->regCount;i++) {
            p_slave->registers[reg_addr + i] = u8_to_u16(pbuf[3 + i*2], pbuf[4 + i*2]);
        }
        return 0; // 成功
    } else if (pbuf[1] == 0x06) {
        reg_addr = u8_to_u16(pbuf[2], pbuf[3]);
        reg_value = u8_to_u16(pbuf[4], pbuf[5]);
        
        if (reg_addr != ptran->regAddr || reg_value != ptran->regValue[0]) {
            return 61;
        }
        return 0; // 成功
    }else if (pbuf[1] == 0x10){
        reg_addr = u8_to_u16(pbuf[2], pbuf[3]);
        reg_count = u8_to_u16(pbuf[4], pbuf[5]);
        if (reg_addr != ptran->regAddr || reg_count != ptran->regCount) {
            return 16;
        }
        return 0; // 成功
    }
    
    return 6;
}

/**
 * @brief Modbus主机状态机，发送p_host->tran_cmd中的命令
 * @return 0-成功，1-从机地址不匹配，2-无响应，3-应答解析失败，4-命令帧构建错误
*/
uint8_t Modbus_Host_Send_Cmd(MODBUS_HostTypeDef* pmb_host, SlaveDevice* p_slave)
{
    uint8_t ret = 0;
    ModbusCommand* p_cmd = &pmb_host->tran_cmd;
    pmb_host->state = MODBUS_STATE_SENDING; // 重置状态

    if (p_cmd->slaveAddr != p_slave->slave_addr ||
        p_cmd->regAddr + p_cmd->regCount > p_slave->reg_count) {
        return 1;
    }
    while (1)
    {
        switch (pmb_host->state)
        {
            case MODBUS_STATE_SENDING:
                if (pmb_host->tran_cmd.funcCode == 0x03) {
                    ret = Modbus_Build_03(p_cmd, pmb_host);
                }else if (pmb_host->tran_cmd.funcCode == 0x06) {
                    ret = Modbus_Build_06(p_cmd, pmb_host);
                } else if (pmb_host->tran_cmd.funcCode == 0x10) {
                    ret = Modbus_Build_10(p_cmd, pmb_host);
                }
                if (ret != 0) {
                    return 4; // 命令帧构建错误
                }
                ret = Modbus_Transmit(pmb_host);
                if (ret != 0) {
                    break; // 发送失败，重试
                }
                ret = Modbus_Wait_RX_flag(100);
                if (ret == 0) { // 没等到
                    pmb_host->miss_count++;
                    pmb_host->state = MODBUS_STATE_ERROR;
                } else {
                    pmb_host->state = MODBUS_STATE_PROCESSING;
                }
                break;
            case MODBUS_STATE_PROCESSING:
                ret = Modbus_Parse_Data(pmb_host, p_slave);
                if (ret == 0) { // 解析成功
                    return 0;
                } else { // 解析失败
                    pmb_host->error_count++;
                    pmb_host->state = MODBUS_STATE_ERROR;
                }
                break;
            case MODBUS_STATE_ERROR:
                if (pmb_host->miss_count >= pmb_host->retry) {
                    return 2; // 多次没等到
                } else if(pmb_host->error_count >= pmb_host->retry) {
                    return 3; // 多次解析失败
                } else {
                    pmb_host->state = MODBUS_STATE_SENDING;break;
                }
        }
    }
}
