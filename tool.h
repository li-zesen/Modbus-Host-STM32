/**
 * @date 2025-11-7
*/

#ifndef __TOOL_H_
#define __TOOL_H_

#include <stddef.h>
#include <stdint.h>

// 两个uint8_t合成uint16_t
static inline uint16_t u8_to_u16(uint8_t high_byte, uint8_t low_byte)
{
    return ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
}

#define GET_u16_HIGH(w)  ((uint8_t)((w) >> 8))
#define GET_u16_LOW(w)   ((uint8_t)((w) & 0xFF))


// 四个uint8_t合成uint32_t
static inline uint32_t u8_to_u32(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0)
{
    return ((uint32_t)byte3 << 24) | 
           ((uint32_t)byte2 << 16) | 
           ((uint32_t)byte1 << 8)  | 
            (uint32_t)byte0;
}


/**
  * @brief  读取uint16_t变量的某一位
  * @param  value: 要读取的变量
  * @param  bit_index:   位位置 (0-15)
  * @retval 位的值 (0或1)
  */
static inline uint8_t Read_Bit_u16(uint16_t value, uint8_t bit_index)
{
    if (bit_index > 15U) return 3;
    return (uint8_t)((value >> bit_index) & 1);
}

/**
  * @brief  写入uint16_t变量的某一位
  * @param  value: 指向要写入的变量的指针
  * @param  bit_index:   位位置 (0-15)
  * @param  state: 0-写0，非0-写1
  */
static inline void Write_Bit_u16(uint16_t* value, uint8_t bit_index, uint8_t state)
{
    if (bit_index > 15 || value == NULL) return;
    
    if (state != 0) {
        *value |= (1 << bit_index);    // 设置位为1
    } else {
        *value &= ~(1 << bit_index);   // 清除位为0
    }
}


// 环形缓冲区结构体
#define RING_BUFFER_SIZE 512
typedef struct ring_buffer{
    uint8_t buffer[RING_BUFFER_SIZE];
    uint16_t write_pos;       // 写指针
    uint16_t read_pos;        // 读指针
}ring_buffer;


uint16_t ring_buffer_copy_all(ring_buffer* rb, uint8_t* dest, uint16_t dest_len);

#endif
