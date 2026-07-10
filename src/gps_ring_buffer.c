/**
* @file gps_ring_buffer.c
 * @brief 基于 Zephyr 4.4.1 原生高速 ring_buf 封装的生产级数据队列实现
 */

#include "gps_ring_buffer.h"
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

// 精确计算 16 帧无损存储所需的总物理内存大小 (16 * 16字节 = 256字节)
#define GPS_RING_BUF_BYTE_SIZE (REQ_CAPACITY * sizeof(gps_location_t))

// 声明一个无任何元数据损耗、完美对齐的 Zephyr 2阶幂原生环形缓冲区
RING_BUF_DECLARE(zephyr_gps_byte_rb, GPS_RING_BUF_BYTE_SIZE
);

void gps_rb_init(void)
{
    ring_buf_reset(&zephyr_gps_byte_rb);
}

int gps_rb_push(const gps_location_t* new_data)
{
    if (new_data == NULL)
    {
        return -EINVAL;
    }

    // 检查缓冲区剩余空闲空间是否足够塞入一帧结构体
    if (ring_buf_space_get(&zephyr_gps_byte_rb) < sizeof(gps_location_t))
    {
        // 缓冲区满（例如消费者完全不存在时），直接安全拒绝写入，保护系统绝对不发生死锁或溢出崩溃
        return -ENOSPC;
    }

    // 直接执行内存拷贝写入环形缓冲区
    int bytes_written = ring_buf_put(&zephyr_gps_byte_rb, (const uint8_t*)new_data, sizeof(gps_location_t));
    if (bytes_written != sizeof(gps_location_t))
    {
        return -EIO;
    }

    return 0;
}

bool gps_rb_pop(gps_location_t* out_data)
{
    if (out_data == NULL)
    {
        return false;
    }

    // 检查缓冲区内是否有足够完整的 1 帧数据供提取
    if (ring_buf_size_get(&zephyr_gps_byte_rb) < sizeof(gps_location_t))
    {
        return false;
    }

    // 从环形队列拉出最早的数据并存入用户指针
    int bytes_read = ring_buf_get(&zephyr_gps_byte_rb, (uint8_t*)out_data, sizeof(gps_location_t));
    if (bytes_read != sizeof(gps_location_t))
    {
        return false;
    }

    return true;
}

uint32_t gps_rb_get_unread_count(void)
{
    // 获取当前积压的总物理字节数，除以单帧结构体大小，精确得出积压帧数
    uint32_t total_unread_bytes = ring_buf_size_get(&zephyr_gps_byte_rb);
    return total_unread_bytes / sizeof(gps_location_t);
}
