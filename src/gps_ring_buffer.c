/**
* @file gps_ring_buffer.c
 * @brief Multi-Instance GPS Ring Buffer Implementation (Safe C17 Compliant)
 */

#include "gps_ring_buffer.h"

void gps_rb_init(gps_rb_instance_t* instance, uint8_t* raw_buffer, uint32_t raw_buffer_size)
{
    if ((instance == NULL) || (raw_buffer == NULL) || (raw_buffer_size < (uint32_t)sizeof(gps_location_t)))
    {
        return;
    }
    // ⭐ Standardized Zephyr way: Delegating internal initialization to the official API safely
    ring_buf_init(&instance->rb_meta, raw_buffer_size, raw_buffer);
}

int gps_rb_push(gps_rb_instance_t* instance, const gps_location_t* new_data)
{
    if ((instance == NULL) || (new_data == NULL))
    {
        return -EINVAL;
    }

    if (ring_buf_space_get(&instance->rb_meta) < (uint32_t)sizeof(gps_location_t))
    {
        return -ENOSPC;
    }

    uint32_t bytes_written = ring_buf_put(&instance->rb_meta, (const uint8_t*)new_data,
                                          (uint32_t)sizeof(gps_location_t));
    if (bytes_written != (uint32_t)sizeof(gps_location_t))
    {
        return -EIO;
    }

    return 0;
}

bool gps_rb_pop(gps_rb_instance_t* instance, gps_location_t* out_data)
{
    if ((instance == NULL) || (out_data == NULL))
    {
        return false;
    }

    if (ring_buf_size_get(&instance->rb_meta) < (uint32_t)sizeof(gps_location_t))
    {
        return false;
    }

    uint32_t bytes_read = ring_buf_get(&instance->rb_meta, (uint8_t*)out_data, (uint32_t)sizeof(gps_location_t));
    if (bytes_read != (uint32_t)sizeof(gps_location_t))
    {
        return false;
    }

    return true;
}

uint32_t gps_rb_get_unread_count(const gps_rb_instance_t* instance)
{
    if (instance == NULL)
    {
        return 0U;
    }
    uint32_t total_unread_bytes = ring_buf_size_get(&instance->rb_meta);
    return total_unread_bytes / (uint32_t)sizeof(gps_location_t);
}
