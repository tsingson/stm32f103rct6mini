#include "walk.h"
#include <stddef.h>

static inline uint32_t walk_isqrt(uint32_t val) {
    uint32_t res = 0U;
    uint32_t bit = 1U << 30;
    while (bit > val) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (val >= res + bit) {
            val -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

static inline int32_t walk_moving_average(walk_pedometer_t *ctx, int32_t new_val) {
    ctx->filter_sum -= ctx->filter_buffer[ctx->filter_index];
    ctx->filter_buffer[ctx->filter_index] = new_val;
    ctx->filter_sum += new_val;
    ctx->filter_index = (ctx->filter_index + 1) % FILTER_WINDOW_SIZE;
    /* 修正：显式强转除数，消除隐式符号类型转换隐患 */
    return ctx->filter_sum / (int32_t)FILTER_WINDOW_SIZE;
}

void walk_pedometer_init(walk_pedometer_t *ctx, uint8_t range_g, uint32_t threshold_lsb, uint32_t delay_ms, uint32_t timeout_ms) {
    if (ctx == NULL) {
        return;
    }

    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        ctx->filter_buffer[i] = 0;
    }
    ctx->filter_index = 0;
    ctx->filter_sum = 0;

    ctx->is_above_threshold = false;
    ctx->last_step_time_ms = -1LL;
    ctx->continuous_steps = 0U;
    ctx->is_active = false;

    ctx->step_threshold = threshold_lsb;
    ctx->step_delay_ms = delay_ms;
    ctx->walk_timeout_ms = timeout_ms;
    ctx->gravity_base = (range_g == 4U) ? 2048U : 4096U;
}

bool walk_pedometer_process(walk_pedometer_t *ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms, uint32_t *steps_to_add) {
    if (ctx == NULL || steps_to_add == NULL) {
        return false;
    }

    *steps_to_add = 0U;

    uint32_t ux = (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
    uint32_t uy = (y < 0) ? (uint32_t)(-y) : (uint32_t)y;
    uint32_t uz = (z < 0) ? (uint32_t)(-z) : (uint32_t)z;
    uint32_t sum_squares = (ux * ux) + (uy * uy) + (uz * uz);

    int32_t vmx = (int32_t)walk_isqrt(sum_squares);
    int32_t vmx_filtered = walk_moving_average(ctx, vmx);

    uint32_t u_vmx_filtered = (vmx_filtered < 0) ? 0U : (uint32_t)vmx_filtered;

    /* 断步超时检测 */
    if (ctx->last_step_time_ms != -1LL) {
        int64_t idle_diff = current_time_ms - ctx->last_step_time_ms;
        if (idle_diff > (int64_t)ctx->walk_timeout_ms) {
            ctx->continuous_steps = 0U;
            ctx->is_active = false;
        }
    }

    uint32_t high_bound = ctx->gravity_base + ctx->step_threshold;
    uint32_t low_bound  = ctx->gravity_base + (ctx->step_threshold / 2U);

    if (u_vmx_filtered > high_bound) {
        if (!ctx->is_above_threshold) {
            int64_t time_diff = current_time_ms - ctx->last_step_time_ms;
            uint64_t elapsed_ms = (time_diff < 0) ? 0ULL : (uint64_t)time_diff;

            if (ctx->last_step_time_ms == -1LL || elapsed_ms > (uint64_t)ctx->step_delay_ms) {
                ctx->last_step_time_ms = current_time_ms;

                if (ctx->is_active) {
                    *steps_to_add = 1U;
                } else {
                    ctx->continuous_steps++;
                    if (ctx->continuous_steps >= WALK_REQUIRED_STEPS) {
                        ctx->is_active = true;
                        *steps_to_add = (uint32_t)WALK_REQUIRED_STEPS;
                    }
                }
            }
            ctx->is_above_threshold = true;
        }
    } else if (u_vmx_filtered < low_bound) {
        ctx->is_above_threshold = false;
    }

    return ctx->is_active;
}
