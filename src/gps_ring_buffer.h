/**
* @file gps_ring_buffer.h
 * @brief 高可靠性通用 GPS 环形缓冲区头文件 (生产交付级，严格符合 C17)
 */

#ifndef GPS_RING_BUFFER_H
#define GPS_RING_BUFFER_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#define REQ_CAPACITY 16U

/* 保留用户核心 GPS 结构体 (16字节) */
typedef struct {
    uint8_t fixType;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t gSpeed;
} gps_location_t;

/**
 * @brief 初始化环形缓冲区
 */
void gps_rb_init(void);

/**
 * @brief 压入新数据。若缓冲区满（如消费者不存在），将安全拒绝写入，绝不崩溃。
 * @return 0 成功, 负数 空间已满或失败
 */
int gps_rb_push(const gps_location_t *new_data);

/**
 * @brief 从缓冲区弹出一帧最早的未读数据 (非阻塞)
 * @return true 成功获取数据, false 缓冲区当前为空
 */
bool gps_rb_pop(gps_location_t *out_data);

/**
 * @brief 获取当前环形缓冲区内剩余的未读数据帧数
 */
uint32_t gps_rb_get_unread_count(void);

#endif // GPS_RING_BUFFER_H
