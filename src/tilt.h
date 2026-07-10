#ifndef TILT_H
#define TILT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Context structure for the Fixed-Point Tilt and Incline Sensor.
 * @note All calculations are done in pure 32-bit integer domain.
 */
typedef struct {
    int32_t base_x;  /* Calibrated X axis baseline (LSB) */
    int32_t base_y;  /* Calibrated Y axis baseline (LSB) */
    int32_t base_z;  /* Calibrated Z axis baseline (LSB) */
    uint32_t base_mag; /* Pre-calculated baseline vector magnitude */
    uint32_t angle_threshold_mdeg; /* Incline threshold in millidegrees (e.g., 10000 = 10.0°) */
} tilt_sensor_t;

/**
 * @brief Initializes the sensor structural configuration.
 */
void tilt_sensor_init(tilt_sensor_t *ctx, uint32_t threshold_mdeg);

/**
 * @brief Sets the static spatial baseline vector from a calibrated array of samples.
 */
void tilt_sensor_set_baseline(tilt_sensor_t *ctx, int16_t x, int16_t y, int16_t z);

/**
 * @brief Calculates the spatial offset angle compared to the baseline vector.
 * @return Computed tilt deviation angle in millidegrees (e.g., 45000 means 45.000°)
 */
uint32_t tilt_sensor_get_angle(const tilt_sensor_t *ctx, int16_t x, int16_t y, int16_t z);

/**
 * @brief Binary categorization logic equivalent to the original isDoorOpen method.
 */
bool tilt_sensor_is_exceeded(const tilt_sensor_t *ctx, uint32_t current_angle_mdeg);

#endif /* TILT_H */
