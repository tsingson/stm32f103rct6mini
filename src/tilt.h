#ifndef TILT_H
#define TILT_H

#include <stdint.h>
#include <stdbool.h>

/* C17 常量表达式显式隔离 */
#define TILT_HISTORY_SIZE     (8)

/**
 * @brief 强健型纯整数空间向量倾角上下文结构体
 */
typedef struct {
    int32_t base_x;
    int32_t base_y;
    int32_t base_z;
    uint32_t base_mag;
    uint32_t angle_threshold_mdeg;

    /* 车载/轮椅微振动拦截缓冲池 */
    uint32_t angle_history[TILT_HISTORY_SIZE];
    int history_index;
    uint32_t history_sum;
    uint32_t transit_suppression_threshold;
} tilt_sensor_t;

void tilt_sensor_init(tilt_sensor_t *ctx, uint32_t threshold_mdeg, uint32_t suppression_mdeg);
void tilt_sensor_set_baseline(tilt_sensor_t *ctx, int16_t x, int16_t y, int16_t z);
uint32_t tilt_sensor_get_angle(tilt_sensor_t *ctx, int16_t x, int16_t y, int16_t z);
bool tilt_sensor_is_exceeded(const tilt_sensor_t *ctx, uint32_t current_angle_mdeg);

#endif /* TILT_H */
