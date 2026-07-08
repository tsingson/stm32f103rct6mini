#ifndef GPS_REPORT_H_
#define GPS_REPORT_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    float latitude;
    float longitude;
    uint32_t timestamp;
    uint8_t sat_count;
    float speed;
    char device_id[32];
} gps_report_t;

gps_report_t* gps_report_create(float lat, float lng, uint32_t ts,
                                uint8_t sat, float spd, const char* dev_id);

void gps_report_destroy(gps_report_t* report);

bool gps_report_serialize(const gps_report_t* report, uint8_t* buffer, size_t* out_len);

bool gps_report_deserialize(const uint8_t* buffer, size_t buf_len, gps_report_t* report);

void gps_report_print(const gps_report_t* report);

#endif /* GPS_REPORT_H_ */
