/**
* @file gps_ring_buffer.h
 * @brief Multi-instance GPS Ring Buffer Core Header (Production-Grade C17 / Zephyr 4.4.1)
 */

#ifndef GPS_RING_BUFFER_H
#define GPS_RING_BUFFER_H

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdint.h>
#include <stdbool.h>

#define REQ_CAPACITY 16U

/* Core 16-byte GPS context representation data structure */
typedef struct {
    uint8_t fixType;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t gSpeed;
} gps_location_t;

#define GPS_RING_BUF_BYTE_SIZE (REQ_CAPACITY * sizeof(gps_location_t))

// Container layout tracking structure instance
typedef struct {
    struct ring_buf rb_meta;
} gps_rb_instance_t;

// ==============================================================================
// ⭐ FIXED COMPLIANT MACRO DEFINE (Zero struct hack, purely allocates memory blocks)
// ==============================================================================
/**
 * @brief Statically allocate tracking objects and array contexts for a new ring buffer instance.
 * @param name The global token identifier of your queue tracker.
 */
#define GPS_RB_INSTANCE_DEFINE(name)                                           \
    uint8_t __aligned(4) name##_raw_buf[GPS_RING_BUF_BYTE_SIZE];               \
    gps_rb_instance_t name

// ==============================================================================
// Function APIs
// ==============================================================================

/**
 * @brief Explicit runtime configuration interface binding raw arrays to structural context blocks
 */
void gps_rb_init(gps_rb_instance_t *instance, uint8_t *raw_buffer, uint32_t raw_buffer_size);

int gps_rb_push(gps_rb_instance_t *instance, const gps_location_t *new_data);
bool gps_rb_pop(gps_rb_instance_t *instance, gps_location_t *out_data);
uint32_t gps_rb_get_unread_count(const gps_rb_instance_t *instance);

#endif // GPS_RING_BUFFER_H
