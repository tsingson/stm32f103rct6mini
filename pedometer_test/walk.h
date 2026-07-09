#ifndef WALK_H
#define WALK_H

#include <stdbool.h>
#include <stdint.h>

/* C17 常量表达式保护 */
#define FILTER_WINDOW_SIZE    (5)
#define WALK_REQUIRED_STEPS   (4U)

/**
 * @brief 纯整数计步器运行上下文结构体（C17 严格强类型版）
 */
typedef struct {
    int32_t filter_buffer[FILTER_WINDOW_SIZE];
    int filter_index;
    int32_t filter_sum;

    bool is_above_threshold;
    int64_t last_step_time_ms;

    uint32_t continuous_steps;
    bool is_active;

    uint32_t step_threshold;
    uint32_t step_delay_ms;
    uint32_t gravity_base;
    uint32_t walk_timeout_ms;
} walk_pedometer_t;

void walk_pedometer_init(walk_pedometer_t *ctx, uint8_t range_g, uint32_t threshold_lsb, uint32_t delay_ms, uint32_t timeout_ms);
bool walk_pedometer_process(walk_pedometer_t *ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms, uint32_t *steps_to_add);

#endif /* WALK_H */
