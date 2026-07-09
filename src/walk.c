#include "walk.h"
#include <stddef.h>

/**
 * @brief 内部函数：快速整数开平方 (基于位移的二分逼近法)
 * 纯无符号 32 位运算，完美规避 C17 溢出风险
 */
static inline uint32_t walk_isqrt(uint32_t val)
{
    uint32_t res = 0;
    uint32_t bit = 1U << 30;

    while (bit > val)
    {
        bit >>= 2;
    }

    while (bit != 0)
    {
        if (val >= res + bit)
        {
            val -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/**
 * @brief 内部函数：纯整数滑动窗口均值滤波
 */
static inline int32_t walk_moving_average(walk_pedometer_t* ctx, int32_t new_val)
{
    ctx->filter_sum -= ctx->filter_buffer[ctx->filter_index];
    ctx->filter_buffer[ctx->filter_index] = new_val;
    ctx->filter_sum += new_val;
    ctx->filter_index = (ctx->filter_index + 1) % FILTER_WINDOW_SIZE;

    return ctx->filter_sum / FILTER_WINDOW_SIZE;
}

void walk_pedometer_init(walk_pedometer_t* ctx, uint8_t range_g, uint32_t threshold_lsb, uint32_t delay_ms)
{
    if (ctx == NULL)
    {
        return;
    }

    for (int i = 0; i < FILTER_WINDOW_SIZE; i++)
    {
        ctx->filter_buffer[i] = 0;
    }
    ctx->filter_index = 0;
    ctx->filter_sum = 0;

    ctx->is_above_threshold = false;
    ctx->last_step_time_ms = 0;

    ctx->step_threshold = threshold_lsb;
    ctx->step_delay_ms = delay_ms;

    /* 假定 LIS3DH 运行在 12-bit 高分辨率模式（1g = 4096 LSB） */
    if (range_g == 4)
    {
        ctx->gravity_base = 2048U;
    }
    else
    {
        ctx->gravity_base = 4096U;
    }
}

bool walk_pedometer_process(walk_pedometer_t* ctx, int16_t x, int16_t y, int16_t z, int64_t current_time_ms)
{
    if (ctx == NULL)
    {
        return false;
    }

    /* 1. 先将有符号取绝对值转换为无符号 uint32_t，彻底根除有符号整型乘法溢出的 Undefined Behavior */
    uint32_t ux = (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
    uint32_t uy = (y < 0) ? (uint32_t)(-y) : (uint32_t)y;
    uint32_t uz = (z < 0) ? (uint32_t)(-z) : (uint32_t)z;

    uint32_t sum_squares = (ux * ux) + (uy * uy) + (uz * uz);

    /* 2. 快速无符号整数开方 */
    int32_t vmx = (int32_t)walk_isqrt(sum_squares);

    /* 3. 整数滑动平滑滤波 */
    int32_t vmx_filtered = walk_moving_average(ctx, vmx);
    bool step_detected = false;

    /* 4. 防抖及阈值判定：对时间差进行防御性边界拦截，排除时间戳倒退等异常 */
    int64_t time_diff = current_time_ms - ctx->last_step_time_ms;
    uint64_t elapsed_ms = (time_diff < 0) ? 0 : (uint64_t)time_diff;

    int32_t high_bound = (int32_t)(ctx->gravity_base + ctx->step_threshold);
    int32_t low_bound = (int32_t)(ctx->gravity_base + (ctx->step_threshold / 2U));

    if (vmx_filtered > high_bound)
    {
        if (!ctx->is_above_threshold)
        {
            if (elapsed_ms > (uint64_t)ctx->step_delay_ms)
            {
                ctx->last_step_time_ms = current_time_ms;
                step_detected = true;
            }
            ctx->is_above_threshold = true;
        }
    }
    else if (vmx_filtered < low_bound)
    {
        ctx->is_above_threshold = false;
    }

    return step_detected;
}
