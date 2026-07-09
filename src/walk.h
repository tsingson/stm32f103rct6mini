#ifndef WALK_H
#define WALK_H

#include <stdbool.h>
#include <stdint.h>

/* C17 常量表达式保护 */
#define FILTER_WINDOW_SIZE  (5)

/**
 * @brief 纯整数计步器运行上下文结构体
 */
typedef struct
{
    /* 滤波相关变量 - 使用纯整数缓冲区 */
    int32_t filter_buffer[FILTER_WINDOW_SIZE];
    int filter_index;
    int32_t filter_sum;

    /* 状态机与防抖变量 */
    bool is_above_threshold;
    int64_t last_step_time_ms;

    /* 纯整数算法配置参数 */
    uint32_t step_threshold; /* 计步原始数据波动阈值 (LSB 差值单位) */
    uint32_t step_delay_ms; /* 两次迈步间的最小合法时间间隔 (ms) */
    uint32_t gravity_base; /* 传感器静止时的标准重力大小 (LSB 单位) */
} walk_pedometer_t;

void walk_pedometer_init(walk_pedometer_t* ctx, uint8_t range_g, uint32_t threshold_lsb, uint32_t delay_ms);

/**
 * @brief 核心处理函数（C17/Zephyr 深度优化版）
 * @note 改为值传递（Value Passing），消除指针解引用开销，天然支持多线程/中断安全
 */
bool walk_pedometer_process(walk_pedometer_t* ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms);

#endif /* WALK_H */
