/**
 * @date 2025-11-7
*/

#include "tool.h"
#include <string.h>

/**
 * @brief 从环形缓冲区复制全部可用数据到目标缓冲区
 * @param rb 环形缓冲区指针
 * @param dest 目标缓冲区指针，必须足够大以容纳所有数据
 * @param dest_len 目标缓冲区长度
 * @return 实际复制的字节数
 */
uint16_t ring_buffer_copy_all(ring_buffer* rb, uint8_t* dest, uint16_t dest_len)
{
    if (rb == NULL || dest == NULL) {
        return 0;
    }
    
    uint16_t data_count;
    uint16_t first_chunk_len;
    
    // 计算当前缓冲区中的数据量
    if (rb->write_pos >= rb->read_pos) {
        data_count = rb->write_pos - rb->read_pos;
    } else {
        data_count = RING_BUFFER_SIZE - rb->read_pos + rb->write_pos;
    }
    
    if (data_count == 0 || data_count > dest_len) {
        return 0;
    }
    
    // 计算从read_pos到缓冲区末尾的数据长度
    first_chunk_len = RING_BUFFER_SIZE - rb->read_pos;
    
    if (data_count <= first_chunk_len) {
        // 数据没有跨越缓冲区边界，一次性复制
        memcpy(dest, &rb->buffer[rb->read_pos], data_count);
    } else {
        // 数据跨越缓冲区边界，分两次复制
        memcpy(dest, &rb->buffer[rb->read_pos], first_chunk_len);
        memcpy(dest + first_chunk_len, rb->buffer, data_count - first_chunk_len);
    }
    // 新的read_pos位置
    rb->read_pos = rb->write_pos;

    return data_count;
}