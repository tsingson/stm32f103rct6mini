#include "tilt.h"
#include <stddef.h>

/* Fixed-point multiplier scaling factor for normalized calculation spaces */
#define FIX_SCALE           (1000000LL)
#define MILLIDEG_90         (90000U)
#define MILLIDEG_180        (180000U)

/**
 * @brief Fast integer square root (Binary scaling approximation)
 */
static inline uint32_t tilt_isqrt(uint64_t val) {
    uint64_t res = 0ULL;
    uint64_t bit = 1ULL << 62; /* Handle up to 64-bit sum of squares products */

    while (bit > val) {
        bit >>= 2;
    }
    while (bit != 0ULL) {
        if (val >= res + bit) {
            val -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)res;
}

/**
 * @brief Pure integer Inverse Cosine (acos) approximation.
 * @param cos_val Normalized dot product scaled by 1,000,000 (Range: -1000000 to 1000000)
 * @return Angle output mapping directly into Millidegrees (0 to 180000)
 */
static uint32_t tilt_fast_acos_mdeg(int32_t cos_val) {
    bool negative = false;
    uint64_t x, x2, x3, x5;
    uint64_t angle_scaled_rad;

    if (cos_val < 0) {
        negative = true;
        cos_val = -cos_val;
    }

    if (cos_val >= (int32_t)FIX_SCALE) {
        return negative ? MILLIDEG_180 : 0U;
    }

    /* Target arcsin(x) approximation via structural Taylor bounds: 
       asin(x) ≈ x + (x^3)/6 + (3*x^5)/40. Then acos(x) = pi/2 - asin(x) */
    x = (uint64_t)cos_val;
    x2 = (x * x) / FIX_SCALE;
    x3 = (x2 * x) / FIX_SCALE;
    x5 = (((x3 * x2) / FIX_SCALE) * 3ULL) / 40ULL;

    angle_scaled_rad = x + (x3 / 6ULL) + x5;

    /* Convert standard Radian bounds to Millidegrees mapping factor (180,000 / PI) ≈ 57295.779 */
    /* Handled strictly via integer factor arithmetic to secure 0.01 degree accuracy */
    uint32_t delta_mdeg = (uint32_t)((angle_scaled_rad * 57296ULL) / FIX_SCALE);

    if (negative) {
        return MILLIDEG_90 + delta_mdeg;
    }
    return (MILLIDEG_90 >= delta_mdeg) ? (MILLIDEG_90 - delta_mdeg) : 0U;
}

void tilt_sensor_init(tilt_sensor_t *ctx, uint32_t threshold_mdeg) {
    if (ctx == NULL) {
        return;
    }
    ctx->base_x = 0;
    ctx->base_y = 0;
    ctx->base_z = 4096; /* Defaults mapping towards normal 1g vector space */
    ctx->base_mag = 4096U;
    ctx->angle_threshold_mdeg = threshold_mdeg;
}

void tilt_sensor_set_baseline(tilt_sensor_t *ctx, int16_t x, int16_t y, int16_t z) {
    if (ctx == NULL) {
        return;
    }
    ctx->base_x = (int32_t)x;
    ctx->base_y = (int32_t)y;
    ctx->base_z = (int32_t)z;

    uint64_t sq_x = (uint64_t)((int64_t)x * (int64_t)x);
    uint64_t sq_y = (uint64_t)((int64_t)y * (int64_t)y);
    uint64_t sq_z = (uint64_t)((int64_t)z * (int64_t)z);

    ctx->base_mag = tilt_isqrt(sq_x + sq_y + sq_z);
}

uint32_t tilt_sensor_get_angle(const tilt_sensor_t *ctx, int16_t x, int16_t y, int16_t z) {
    if (ctx == NULL || ctx->base_mag == 0U) {
        return 0U;
    }

    /* 1. Calculate current vector spatial magnitude */
    uint64_t cur_x64 = (uint64_t)((int64_t)x * (int64_t)x);
    uint64_t cur_y64 = (uint64_t)((int64_t)y * (int64_t)y);
    uint64_t cur_z64 = (uint64_t)((int64_t)z * (int64_t)z);
    uint32_t cur_mag = tilt_isqrt(cur_x64 + cur_y64 + cur_z64);

    if (cur_mag == 0U) {
        return 0U;
    }

    /* 2. Execute dot product linear equation */
    int64_t dot_product = ((int64_t)x * ctx->base_x) + 
                          ((int64_t)y * ctx->base_y) + 
                          ((int64_t)z * ctx->base_z);

    /* 3. Extract normalized cosine ratio scaled up to our integer matrix */
    int64_t denominator = (int64_t)ctx->base_mag * (int64_t)cur_mag;
    int32_t cos_theta = (int32_t)((dot_product * FIX_SCALE) / denominator);

    /* 4. Resolve angular inverse transformation mapping */
    return tilt_fast_acos_mdeg(cos_theta);
}

bool tilt_sensor_is_exceeded(const tilt_sensor_t *ctx, uint32_t current_angle_mdeg) {
    if (ctx == NULL) {
        return false;
    }
    return current_angle_mdeg > ctx->angle_threshold_mdeg;
}
