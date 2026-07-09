#ifndef WALK_H
#define WALK_H

#include <stdbool.h>
#include <stdint.h>

#define FILTER_WINDOW_SIZE  (5)
#define WALK_REQUIRED_STEPS (4U)     /* 显式无符号整型后缀 */
#define WALK_TIMEOUT_MS     (2000LL) /* 显式 64 位长整型后缀，匹配 Zephyr 时间戳 */

typedef struct {
    int32_t filter_buffer[FILTER_WINDOW_SIZE];
    int filter_index;
    int32_t filter_sum;

    bool is_above_threshold;
    int64_t last_step_time_ms; /* 系统上电初值处理 */

    uint32_t continuous_steps;
    bool is_active;

    uint32_t step_threshold;
    uint32_t step_delay_ms;
    uint32_t gravity_base;
} walk_pedometer_t;

void walk_pedometer_init(walk_pedometer_t *ctx, uint8_t range_g, uint32_t threshold_lsb, uint32_t delay_ms);
bool walk_pedometer_process(walk_pedometer_t *ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms, uint32_t *steps_to_add);

#endif /* WALK_H */
