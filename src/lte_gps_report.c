#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include "lte_gps_report.h"

#define GPS_PAYLOAD_MIN_LEN  17

gps_report_t* gps_report_create(float lat, float lng, uint32_t ts,
                                uint8_t sat, float spd, const char* dev_id)
{
    if (dev_id == NULL)
    {
        return NULL;
    }

    gps_report_t* report = k_malloc(sizeof(gps_report_t));
    if (report == NULL)
    {
        return NULL;
    }

    report->latitude = lat;
    report->longitude = lng;
    report->timestamp = ts;
    report->sat_count = sat;
    report->speed = spd;
    strncpy(report->device_id, dev_id, sizeof(report->device_id) - 1U);
    report->device_id[sizeof(report->device_id) - 1U] = '\0';

    return report;
}

void gps_report_destroy(gps_report_t* report)
{
    if (report != NULL)
    {
        k_free(report);
    }
}

bool gps_report_serialize(const gps_report_t* report, uint8_t* buffer, size_t* out_len)
{
    if (report == NULL || buffer == NULL || out_len == NULL)
    {
        return false;
    }

    size_t id_len = strlen(report->device_id);
    size_t total_len = GPS_PAYLOAD_MIN_LEN + id_len;

    if (total_len > 256U)
    {
        return false;
    }

    uint32_t lat_bits;
    uint32_t lng_bits;
    uint32_t spd_bits;

    memcpy(&lat_bits, &report->latitude, sizeof(lat_bits));
    memcpy(&lng_bits, &report->longitude, sizeof(lng_bits));
    memcpy(&spd_bits, &report->speed, sizeof(spd_bits));

    buffer[0] = (lat_bits >> 24) & 0xFFU;
    buffer[1] = (lat_bits >> 16) & 0xFFU;
    buffer[2] = (lat_bits >> 8) & 0xFFU;
    buffer[3] = lat_bits & 0xFFU;

    buffer[4] = (lng_bits >> 24) & 0xFFU;
    buffer[5] = (lng_bits >> 16) & 0xFFU;
    buffer[6] = (lng_bits >> 8) & 0xFFU;
    buffer[7] = lng_bits & 0xFFU;

    buffer[8] = (report->timestamp >> 24) & 0xFFU;
    buffer[9] = (report->timestamp >> 16) & 0xFFU;
    buffer[10] = (report->timestamp >> 8) & 0xFFU;
    buffer[11] = report->timestamp & 0xFFU;

    buffer[12] = report->sat_count;

    buffer[13] = (spd_bits >> 24) & 0xFFU;
    buffer[14] = (spd_bits >> 16) & 0xFFU;
    buffer[15] = (spd_bits >> 8) & 0xFFU;
    buffer[16] = spd_bits & 0xFFU;

    memcpy(&buffer[17], report->device_id, id_len);

    *out_len = total_len;
    return true;
}

bool gps_report_deserialize(const uint8_t* buffer, size_t buf_len, gps_report_t* report)
{
    if (buffer == NULL || report == NULL || buf_len < GPS_PAYLOAD_MIN_LEN)
    {
        return false;
    }

    uint32_t lat_bits = ((uint32_t)buffer[0] << 24) |
        ((uint32_t)buffer[1] << 16) |
        ((uint32_t)buffer[2] << 8) |
        ((uint32_t)buffer[3]);
    memcpy(&report->latitude, &lat_bits, sizeof(report->latitude));

    uint32_t lng_bits = ((uint32_t)buffer[4] << 24) |
        ((uint32_t)buffer[5] << 16) |
        ((uint32_t)buffer[6] << 8) |
        ((uint32_t)buffer[7]);
    memcpy(&report->longitude, &lng_bits, sizeof(report->longitude));

    report->timestamp = ((uint32_t)buffer[8] << 24) |
        ((uint32_t)buffer[9] << 16) |
        ((uint32_t)buffer[10] << 8) |
        ((uint32_t)buffer[11]);

    report->sat_count = buffer[12];

    uint32_t spd_bits = ((uint32_t)buffer[13] << 24) |
        ((uint32_t)buffer[14] << 16) |
        ((uint32_t)buffer[15] << 8) |
        ((uint32_t)buffer[16]);
    memcpy(&report->speed, &spd_bits, sizeof(report->speed));

    size_t id_len = buf_len - GPS_PAYLOAD_MIN_LEN;
    if (id_len >= sizeof(report->device_id))
    {
        id_len = sizeof(report->device_id) - 1U;
    }
    memcpy(report->device_id, &buffer[17], id_len);
    report->device_id[id_len] = '\0';

    return true;
}

void gps_report_print(const gps_report_t* report)
{
    if (report == NULL)
    {
        return;
    }

    printk("[4G-L] 纬度: %.6f | 经度: %.6f\n", (double)report->latitude, (double)report->longitude);
    printk("[4G-L] 搜星数: %d | 速度: %.4f Km/h\n", report->sat_count, (double)report->speed);
    printk("[4G-L] 时间戳: %u | 设备号: %s\n", report->timestamp, report->device_id);
}
