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
    return ctx->filter_sum / (int32_t)FILTER_WINDOW_SIZE;
}

void walk_pedometer_init(walk_pedometer_t *ctx, uint8_t range_g, uint32_t delay_ms, uint32_t timeout_ms, uint32_t min_amplitude) {
    if (ctx == NULL) {
        return;
    }

    /* 初始化平滑滤波器 */
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        ctx->filter_buffer[i] = 0;
    }
    ctx->filter_index = 0;
    ctx->filter_sum = 0;

    /* 初始化重力基准 */
    ctx->gravity_base = (range_g == 4U) ? 2048U : 4096U;

    /* 初始化自适应缓冲区 */
    for (int i = 0; i < ADAPTIVE_WINDOW_SIZE; i++) {
        ctx->adapt_buffer[i] = (int32_t)ctx->gravity_base;
    }
    ctx->adapt_index = 0; /* 修正：显式强行将数组索引归零，彻底根除冷启动越界硬崩溃 */

    /* 初始化状态机与防抖变量 */
    ctx->is_above_threshold = false;
    ctx->last_step_time_ms = -1LL;
    ctx->continuous_steps = 0U;
    ctx->is_active = false;

    /* 加载外部公开配置 */
    ctx->step_delay_ms = delay_ms;
    ctx->walk_timeout_ms = timeout_ms;
    ctx->min_valid_amplitude = min_amplitude;
}

bool walk_pedometer_process(walk_pedometer_t *ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms, uint32_t *steps_to_add) {
    if (ctx == NULL || steps_to_add == NULL) {
        return false;
    }
    *steps_to_add = 0U;

    /* 1. 纯无符号域合成物理模长，规避有符号乘法溢出未定义风险 */
    uint32_t ux = (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
    uint32_t uy = (y < 0) ? (uint32_t)(-y) : (uint32_t)y;
    uint32_t uz = (z < 0) ? (uint32_t)(-z) : (uint32_t)z;
    uint32_t sum_squares = (ux * ux) + (uy * uy) + (uz * uz);
    int32_t vmx = (int32_t)walk_isqrt(sum_squares);
    int32_t vmx_filtered = walk_moving_average(ctx, vmx);

    /* 2. 滚动推入 2 秒动态最大最小追踪池 */
    ctx->adapt_buffer[ctx->adapt_index] = vmx_filtered;
    ctx->adapt_index = (ctx->adapt_index + 1) % ADAPTIVE_WINDOW_SIZE;

    /* 修正：由原先错误的指针隐式赋值，修改为正确解引用数组第0个成员的值 */
    int32_t dynamic_max = ctx->adapt_buffer[0];
    int32_t dynamic_min = ctx->adapt_buffer[0];
    for (int i = 1; i < ADAPTIVE_WINDOW_SIZE; i++) {
        if (ctx->adapt_buffer[i] > dynamic_max) {
            dynamic_max = ctx->adapt_buffer[i];
        }
        if (ctx->adapt_buffer[i] < dynamic_min) {
            dynamic_min = ctx->adapt_buffer[i];
        }
    }

    int32_t dynamic_amplitude = dynamic_max - dynamic_min;

    /* 3. 断步超时检测 */
    if (ctx->last_step_time_ms != -1LL) {
        int64_t idle_diff = current_time_ms - ctx->last_step_time_ms;
        if (idle_diff > (int64_t)ctx->walk_timeout_ms) {
            ctx->continuous_steps = 0U;
            ctx->is_active = false;
        }
    }

    /* 4. 强类型安全：全程采用正向标量推导自适应门槛 */
    uint32_t u_vmx_filtered = (vmx_filtered < 0) ? 0U : (uint32_t)vmx_filtered;
    uint32_t u_dynamic_min  = (dynamic_min < 0) ? 0U : (uint32_t)dynamic_min;
    uint32_t u_amplitude    = (dynamic_amplitude < 0) ? 0U : (uint32_t)dynamic_amplitude;

    uint32_t dynamic_threshold = u_dynamic_min + (u_amplitude / 2U);
    uint32_t dynamic_low_bound = u_dynamic_min + (u_amplitude / 4U);

    /* 5. 自适应动态波峰判定 */
    if (dynamic_amplitude > (int32_t)ctx->min_valid_amplitude) {
        if (u_vmx_filtered > dynamic_threshold) {
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
        } else if (u_vmx_filtered < dynamic_low_bound) {
            ctx->is_above_threshold = false;
        }
    } else {
        /* 💡 修复：阻尼拦截的 else 分支（强制熔断重置） */
        /* 当 2 秒内活动幅度坍塌至噪声范围（完全没动）时，执行全面清空： */
        ctx->is_above_threshold = false;  // 重置波峰触发标志
        ctx->continuous_steps    = 0U;     // 必须清空连续潜在步数，防止残留
        ctx->is_active           = false;  // 必须强制退出行走激活状态，防止静止后再次爆出“追加4步”
    }

    return ctx->is_active;
}
