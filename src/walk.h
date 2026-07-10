#ifndef WALK_H
#define WALK_H

#include <stdbool.h>
#include <stdint.h>

/* C17 标准常量表达式及括号保护 */
#define FILTER_WINDOW_SIZE          (5)
#define ADAPTIVE_WINDOW_SIZE        (50)
#define TILT_HISTORY_SIZE           (8)
#define WALK_REQUIRED_STEPS         (4U)

/**
 * @brief 动态自适应计步 + 空间空间倾角方差防坐车合并版上下文结构体（C17 修复版）
 */
typedef struct {
    /* 1. 基础平滑滤波单元 */
    int32_t filter_buffer[FILTER_WINDOW_SIZE];
    int filter_index;
    int32_t filter_sum;

    /* 2. 动态自适应均值追踪 */
    int32_t adapt_buffer[ADAPTIVE_WINDOW_SIZE];
    int adapt_index;

    /* 3. 空间空间向量倾角与车载微方差拦截器 */
    int32_t base_x;
    int32_t base_y;
    int32_t base_z;
    uint32_t base_mag;
    uint32_t angle_history[TILT_HISTORY_SIZE];
    int tilt_index;
    uint32_t tilt_sum;

    /* 4. 算法业务状态机控制量 */
    bool is_above_threshold;
    int64_t last_step_time_ms;
    uint32_t continuous_steps;
    bool is_active;

    /* 5. 全动态可调生理特征参数 */
    uint32_t step_delay_ms;
    uint32_t walk_timeout_ms;
    uint32_t min_valid_amplitude;
    uint32_t tilt_threshold_mdeg;
    uint32_t transit_suppression;
    uint32_t gravity_base;
} walk_combined_pedometer_t;

void walk_combined_init(walk_combined_pedometer_t *ctx, uint8_t range_g,
                        uint32_t delay_ms, uint32_t timeout_ms, uint32_t min_amplitude,
                        uint32_t tilt_mdeg, uint32_t suppression_mdeg2);

void walk_combined_set_tilt_baseline(walk_combined_pedometer_t *ctx, int16_t x, int16_t y, int16_t z);

bool walk_combined_process(walk_combined_pedometer_t *ctx, int16_t x, int16_t y, int16_t z,
                           int64_t current_time_ms, uint32_t *steps_to_add);

#endif /* WALK_H */
