#ifndef WALK_H
#define WALK_H

#include <stdbool.h>
#include <stdint.h>

/* C17 标准常量表达式及括号隔离保护 */
#define FILTER_WINDOW_SIZE      (5)
#define ADAPTIVE_WINDOW_SIZE    (50)    /* 2秒动态窗口 (25Hz * 2秒 = 50帧) */
#define WALK_REQUIRED_STEPS     (4U)

/**
 * @brief 动态自适应均值计步器运行上下文结构体（C17 工业级修复版）
 */
typedef struct {
    /* 1. 基础平滑滤波缓冲区 */
    int32_t filter_buffer[FILTER_WINDOW_SIZE];
    int filter_index;
    int32_t filter_sum;

    /* 2. 2秒动态自适应最大/最小窗口缓冲区 */
    int32_t adapt_buffer[ADAPTIVE_WINDOW_SIZE];
    int adapt_index;

    /* 3. 状态机与防抖变量 */
    bool is_above_threshold;
    int64_t last_step_time_ms;

    /* 4. 防误判机制变量 */
    uint32_t continuous_steps;
    bool is_active;

    /* 5. 开放配置参数 */
    uint32_t step_delay_ms;
    uint32_t gravity_base;
    uint32_t walk_timeout_ms;
    uint32_t min_valid_amplitude;
} walk_pedometer_t;

/**
 * @brief 初始化自适应计步器
 */
void walk_pedometer_init(walk_pedometer_t *ctx, uint8_t range_g, uint32_t delay_ms, uint32_t timeout_ms, uint32_t min_amplitude);

/**
 * @brief 全自动动态波峰波谷自适应检测处理函数
 */
bool walk_pedometer_process(walk_pedometer_t *ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms, uint32_t *steps_to_add);

#endif /* WALK_H */
