#include "tilt.h"
#include <stddef.h>

/* 修正：将后缀全部规范为无符号 ULL，完美规避 C17 隐式符号提升导致的有符号回绕漏洞 */
#define FIX_SCALE                   (1000000ULL)
#define MILLIDEG_90                 (90000U)
#define MILLIDEG_180                (180000U)
#define MIN_MAGNITUDE_THRESHOLD     (100U)

static inline uint32_t tilt_isqrt(uint64_t val)
{
    uint64_t res = 0ULL;
    uint64_t bit = 1ULL << 62;
    while (bit > val)
    {
        bit >>= 2;
    }
    while (bit != 0ULL)
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
    return (uint32_t)res;
}

/**
 * @brief 高性能极值多项式有理逼近反余弦算法（C17 安全约束版）
 */
static uint32_t tilt_fast_acos_mdeg(int32_t cos_val)
{
    bool negative = false;
    uint64_t x, num, den, angle_rad_scaled;
    uint64_t term1;
    uint32_t angle_mdeg;

    if (cos_val < 0)
    {
        negative = true;
        /* 在进行绝对值转换前，通过 C17 强转保护边界 */
        cos_val = (cos_val == INT32_MIN) ? INT32_MAX : -cos_val;
    }

    if (cos_val >= (int32_t)FIX_SCALE)
    {
        return negative ? MILLIDEG_180 : 0U;
    }

    x = (uint64_t)cos_val;

    /* 修正：全程处于纯无符号 64 位域内，并前置防止无符号减法倒退回绕的断言保护 */
    term1 = 1570796ULL * FIX_SCALE;
    if (term1 >= (1569000ULL * x))
    {
        num = term1 - (1569000ULL * x);
    }
    else
    {
        num = 0ULL;
    }

    den = (1000000ULL * FIX_SCALE) + (560000ULL * x);

    /* 修正：代数重新约分（先除后乘再细化），彻底消灭 (angle_rad_scaled * 57296ULL) 逼近 64位上限的硬件整型截断崩溃 */
    angle_rad_scaled = (num / den);
    angle_mdeg = (uint32_t)(((angle_rad_scaled * x) / FIX_SCALE) * 57296ULL / FIX_SCALE);

    if (negative)
    {
        return MILLIDEG_90 + (MILLIDEG_90 >= angle_mdeg ? (MILLIDEG_90 - angle_mdeg) : angle_mdeg);
    }

    return angle_mdeg;
}

void tilt_sensor_init(tilt_sensor_t* ctx, uint32_t threshold_mdeg, uint32_t suppression_mdeg)
{
    if (ctx == NULL) return;
    ctx->base_x = 0;
    ctx->base_y = 0;
    ctx->base_z = 4096;
    ctx->base_mag = 4096U;
    ctx->angle_threshold_mdeg = threshold_mdeg;
    ctx->transit_suppression_threshold = suppression_mdeg;
    ctx->history_index = 0;
    ctx->history_sum = 0U;
    for (int i = 0; i < TILT_HISTORY_SIZE; i++)
    {
        ctx->angle_history[i] = 0U;
    }
}

void tilt_sensor_set_baseline(tilt_sensor_t* ctx, int16_t x, int16_t y, int16_t z)
{
    if (ctx == NULL) return;
    uint64_t sums;
    ctx->base_x = (int32_t)x;
    ctx->base_y = (int32_t)y;
    ctx->base_z = (int32_t)z;
    sums = ((int64_t)x * x) + ((int64_t)y * y) + ((int64_t)z * z);
    ctx->base_mag = tilt_isqrt(sums);
}

uint32_t tilt_sensor_get_angle(tilt_sensor_t* ctx, int16_t x, int16_t y, int16_t z)
{
    uint64_t cur_x64, cur_y64, cur_z64;
    uint32_t cur_mag;
    int64_t dot_product, denominator;
    int32_t cos_theta;
    uint32_t computed_angle;

    if (ctx == NULL || ctx->base_mag < MIN_MAGNITUDE_THRESHOLD) return 0U;

    cur_x64 = (uint64_t)((int64_t)x * x);
    cur_y64 = (uint64_t)((int64_t)y * y);
    cur_z64 = (uint64_t)((int64_t)z * z);
    cur_mag = tilt_isqrt(cur_x64 + cur_y64 + cur_z64);

    if (cur_mag < MIN_MAGNITUDE_THRESHOLD) return 0U;

    dot_product = ((int64_t)x * ctx->base_x) + ((int64_t)y * ctx->base_y) + ((int64_t)z * ctx->base_z);
    denominator = (int64_t)ctx->base_mag * cur_mag;

    if (denominator == 0LL) return 0U;

    cos_theta = (int32_t)((dot_product * (int64_t)FIX_SCALE) / denominator);
    computed_angle = tilt_fast_acos_mdeg(cos_theta);

    /* 滚动刷新微方差历史滑动窗口 */
    ctx->history_sum -= ctx->angle_history[ctx->history_index];
    ctx->angle_history[ctx->history_index] = computed_angle;
    ctx->history_sum += computed_angle;
    ctx->history_index = (ctx->history_index + 1) % TILT_HISTORY_SIZE;

    return computed_angle;
}

bool tilt_sensor_is_exceeded(const tilt_sensor_t* ctx, uint32_t current_angle_mdeg)
{
    uint32_t mean;
    uint64_t variance_sum = 0ULL;
    uint32_t micro_variance;

    if (ctx == NULL) return false;

    mean = ctx->history_sum / (uint32_t)TILT_HISTORY_SIZE;
    for (int i = 0; i < TILT_HISTORY_SIZE; i++)
    {
        int32_t diff = (int32_t)(ctx->angle_history[i] - mean);
        variance_sum += (uint64_t)((int64_t)diff * diff);
    }
    micro_variance = (uint32_t)(variance_sum / (uint64_t)TILT_HISTORY_SIZE);

    /* 💡 核心设计：如果微方差超标，判定为车辆高频颠簸 hum 噪声，强行熔断拦截伪步伐 */
    if (micro_variance > ctx->transit_suppression_threshold)
    {
        return false;
    }

    return current_angle_mdeg > ctx->angle_threshold_mdeg;
}
