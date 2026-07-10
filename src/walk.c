#include "walk.h"
#include <stddef.h>

/* C17 强类型无符号常量后缀修饰 */
#define FIX_SCALE                   (1000000ULL)
#define MILLIDEG_90                 (90000U)
#define MILLIDEG_180                (180000U)
#define MIN_MAGNITUDE_THRESHOLD     (100U)

static inline uint32_t walk_internal_isqrt(uint64_t val)
{
    uint64_t res = 0ULL;
    uint64_t bit = 1ULL << 62;
    while (bit > val) { bit >>= 2; }
    while (bit != 0ULL)
    {
        if (val >= res + bit)
        {
            val -= res + bit;
            res = (res >> 1) + bit;
        }
        else { res >>= 1; }
        bit >>= 2;
    }
    return (uint32_t)res;
}

static uint32_t walk_internal_acos_mdeg(int32_t cos_val)
{
    bool negative = false;
    uint64_t x, num, den, angle_rad_scaled;
    uint64_t term1;
    uint32_t angle_mdeg;

    if (cos_val < 0)
    {
        negative = true;
        cos_val = (cos_val == INT32_MIN) ? INT32_MAX : -cos_val;
    }
    if (cos_val >= (int32_t)FIX_SCALE)
    {
        return negative ? MILLIDEG_180 : 0U;
    }

    x = (uint64_t)cos_val;
    term1 = 1570796ULL * FIX_SCALE;
    num = (term1 >= (1569000ULL * x)) ? (term1 - (1569000ULL * x)) : 0ULL;
    den = (1000000ULL * FIX_SCALE) + (560000ULL * x);

    angle_rad_scaled = (num / den);
    angle_mdeg = (uint32_t)(((angle_rad_scaled * x) / FIX_SCALE) * 57296ULL / FIX_SCALE);

    if (negative)
    {
        return MILLIDEG_90 + (MILLIDEG_90 >= angle_mdeg ? (MILLIDEG_90 - angle_mdeg) : angle_mdeg);
    }
    return angle_mdeg;
}

void walk_combined_init(walk_combined_pedometer_t* ctx, uint8_t range_g,
                        uint32_t delay_ms, uint32_t timeout_ms, uint32_t min_amplitude,
                        uint32_t tilt_mdeg, uint32_t suppression_mdeg2)
{
    if (ctx == NULL) return;

    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) ctx->filter_buffer[i] = 0;
    ctx->filter_index = 0;
    ctx->filter_sum = 0;

    ctx->gravity_base = (range_g == 4U) ? 2048U : 4096U;

    for (int i = 0; i < ADAPTIVE_WINDOW_SIZE; i++) ctx->adapt_buffer[i] = (int32_t)ctx->gravity_base;
    ctx->adapt_index = 0;

    ctx->base_x = 0;
    ctx->base_y = 0;
    ctx->base_z = (int32_t)ctx->gravity_base;
    ctx->base_mag = ctx->gravity_base;
    for (int i = 0; i < TILT_HISTORY_SIZE; i++) ctx->angle_history[i] = 0U;
    ctx->tilt_index = 0;
    ctx->tilt_sum = 0U;

    ctx->is_above_threshold = false;
    ctx->last_step_time_ms = -1LL;
    ctx->continuous_steps = 0U;
    ctx->is_active = false;

    ctx->step_delay_ms = delay_ms;
    ctx->walk_timeout_ms = timeout_ms;
    ctx->min_valid_amplitude = min_amplitude;
    ctx->tilt_threshold_mdeg = tilt_mdeg;
    ctx->transit_suppression = suppression_mdeg2;
}

void walk_combined_set_tilt_baseline(walk_combined_pedometer_t* ctx, int16_t x, int16_t y, int16_t z)
{
    if (ctx == NULL) return;
    ctx->base_x = (int32_t)x;
    ctx->base_y = (int32_t)y;
    ctx->base_z = (int32_t)z;
    uint64_t sums = ((int64_t)x * x) + ((int64_t)y * y) + ((int64_t)z * z);
    ctx->base_mag = walk_internal_isqrt(sums);
}

bool walk_combined_process(walk_combined_pedometer_t* ctx, int16_t x, int16_t y, int16_t z,
                           int64_t current_time_ms, uint32_t* steps_to_add)
{
    if (ctx == NULL || steps_to_add == NULL) return false;
    *steps_to_add = 0U;

    /* ----------------------------------------------------------------- */
    /* STAGE 1: 物理模长解算与自适应自适应滑动窗口更新                        */
    /* ----------------------------------------------------------------- */
    uint32_t ux = (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
    uint32_t uy = (y < 0) ? (uint32_t)(-y) : (uint32_t)y;
    uint32_t uz = (z < 0) ? (uint32_t)(-z) : (uint32_t)z;
    uint64_t sum_squares = ((uint64_t)ux * ux) + ((uint64_t)uy * uy) + ((uint64_t)uz * uz);
    int32_t vmx = (int32_t)walk_internal_isqrt(sum_squares);

    ctx->filter_sum -= ctx->filter_buffer[ctx->filter_index];
    ctx->filter_buffer[ctx->filter_index] = vmx;
    ctx->filter_sum += vmx;
    ctx->filter_index = (ctx->filter_index + 1) % FILTER_WINDOW_SIZE;
    int32_t vmx_filtered = ctx->filter_sum / (int32_t)FILTER_WINDOW_SIZE;

    ctx->adapt_buffer[ctx->adapt_index] = vmx_filtered;
    ctx->adapt_index = (ctx->adapt_index + 1) % ADAPTIVE_WINDOW_SIZE;

    /* 💡 修正 1：由错误的数组指针赋值，修正为正确解引用数组第 0 个数的值 */
    int32_t dynamic_max = ctx->adapt_buffer[0];
    int32_t dynamic_min = ctx->adapt_buffer[0];
    for (int i = 1; i < ADAPTIVE_WINDOW_SIZE; i++)
    {
        if (ctx->adapt_buffer[i] > dynamic_max) dynamic_max = ctx->adapt_buffer[i];
        if (ctx->adapt_buffer[i] < dynamic_min) dynamic_min = ctx->adapt_buffer[i];
    }
    int32_t dynamic_amplitude = dynamic_max - dynamic_min;

    /* ----------------------------------------------------------------- */
    /* STAGE 2: 空间三轴倾角与车载微方差计算机制                           */
    /* ----------------------------------------------------------------- */
    uint32_t computed_angle = 0U;
    uint32_t micro_variance = 0U;

    if (ctx->base_mag >= MIN_MAGNITUDE_THRESHOLD)
    {
        uint32_t cur_mag = walk_internal_isqrt(sum_squares);
        if (cur_mag >= MIN_MAGNITUDE_THRESHOLD)
        {
            int64_t dot_product = ((int64_t)x * ctx->base_x) + ((int64_t)y * ctx->base_y) + ((int64_t)z * ctx->base_z);
            int64_t denominator = (int64_t)ctx->base_mag * cur_mag;
            if (denominator != 0LL)
            {
                int32_t cos_theta = (int32_t)((dot_product * (int64_t)FIX_SCALE) / denominator);
                computed_angle = walk_internal_acos_mdeg(cos_theta);
            }
        }
    }

    ctx->tilt_sum -= ctx->angle_history[ctx->tilt_index];
    ctx->angle_history[ctx->tilt_index] = computed_angle;
    ctx->tilt_sum += computed_angle;
    ctx->tilt_index = (ctx->tilt_index + 1) % TILT_HISTORY_SIZE;

    uint32_t tilt_mean = ctx->tilt_sum / (uint32_t)TILT_HISTORY_SIZE;
    uint64_t variance_accumulator = 0ULL;
    for (int i = 0; i < TILT_HISTORY_SIZE; i++)
    {
        /* 💡 修正 2：通过前置大小断言判定绝对值差，彻底粉碎 C17 无符号域回绕引发的方差爆表 */
        uint32_t u_diff = (ctx->angle_history[i] >= tilt_mean)
                              ? (ctx->angle_history[i] - tilt_mean)
                              : (tilt_mean - ctx->angle_history[i]);
        variance_accumulator += ((uint64_t)u_diff * u_diff);
    }
    micro_variance = (uint32_t)(variance_accumulator / (uint64_t)TILT_HISTORY_SIZE);

    /* ----------------------------------------------------------------- */
    /* STAGE 3: 走走停停超时判定与坐车状态硬熔断检测                       */
    /* ----------------------------------------------------------------- */
    if (ctx->last_step_time_ms != -1LL)
    {
        int64_t idle_diff = current_time_ms - ctx->last_step_time_ms;
        if (idle_diff > (int64_t)ctx->walk_timeout_ms)
        {
            ctx->continuous_steps = 0U;
            ctx->is_active = false;
        }
    }

    /* 🛡️ 乘车/轮椅高频碎震拦截硬熔断 */
    if (micro_variance > ctx->transit_suppression)
    {
        ctx->is_above_threshold = false;
        ctx->continuous_steps = 0U;
        ctx->is_active = false;
        return false;
    }

    /* ----------------------------------------------------------------- */
    /* STAGE 4: 全自动阈值中线判定与 4 步追偿状态机                         */
    /* ----------------------------------------------------------------- */
    uint32_t u_vmx_filtered = (vmx_filtered < 0) ? 0U : (uint32_t)vmx_filtered;
    uint32_t u_dynamic_min = (dynamic_min < 0) ? 0U : (uint32_t)dynamic_min;
    uint32_t u_amplitude = (dynamic_amplitude < 0) ? 0U : (uint32_t)dynamic_amplitude;

    uint32_t dynamic_threshold = u_dynamic_min + (u_amplitude / 2U);
    uint32_t dynamic_low_bound = u_dynamic_min + (u_amplitude / 4U);

    /* 💡 修正 3：全程收拢到纯无符号域进行阻尼和门槛判定，消除隐式符号提升警告 */
    if (u_amplitude > ctx->min_valid_amplitude)
    {
        if (u_vmx_filtered > dynamic_threshold)
        {
            if (!ctx->is_above_threshold)
            {
                int64_t time_diff = current_time_ms - ctx->last_step_time_ms;
                uint64_t elapsed_ms = (time_diff < 0) ? 0ULL : (uint64_t)time_diff;

                if (ctx->last_step_time_ms == -1LL || elapsed_ms > (uint64_t)ctx->step_delay_ms)
                {
                    ctx->last_step_time_ms = current_time_ms;

                    if (ctx->is_active)
                    {
                        *steps_to_add = 1U;
                    }
                    else
                    {
                        ctx->continuous_steps++;
                        if (ctx->continuous_steps >= WALK_REQUIRED_STEPS)
                        {
                            ctx->is_active = true;
                            *steps_to_add = (uint32_t)WALK_REQUIRED_STEPS;
                        }
                    }
                }
                ctx->is_above_threshold = true;
            }
        }
        else if (u_vmx_filtered < dynamic_low_bound)
        {
            ctx->is_above_threshold = false;
        }
    }
    else
    {
        /* 阻尼拦截：幅度坍塌，强制熔断清空，防止幽灵爆发 */
        ctx->is_above_threshold = false;
        ctx->continuous_steps = 0U;
        ctx->is_active = false;
    }

    return ctx->is_active;
}
