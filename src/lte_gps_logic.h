#ifndef LTE_GPS_LOGIC_H_
#define LTE_GPS_LOGIC_H_

#include <stdint.h>
#include <stdbool.h>
#include "lte_gps_report.h"

typedef struct
{
    uint8_t status; // 0:睡眠 1:普通 2:重要工作 3:部分失效
    uint8_t battery; // 0-100%
    uint16_t sleep_countdown; // 秒
    char device_id[32];
} heartbeat_report_t;

heartbeat_report_t* heartbeat_create(uint8_t status, uint8_t battery,
                                     uint16_t sleep_time, const char* dev_id);
void heartbeat_destroy(heartbeat_report_t* hb);
bool heartbeat_serialize(const heartbeat_report_t* hb, uint8_t* buffer, size_t* out_len);

bool ntp_sync(uint32_t * out_timestamp);

bool gps_report_send(const gps_report_t* report, const char* server_host, uint16_t server_port);

bool heartbeat_send(const heartbeat_report_t* hb, const char* server_host, uint16_t server_port);

uint32_t get_system_timestamp(void);

void set_system_timestamp(uint32_t ts);

#endif /* LTE_GPS_LOGIC_H_ */
