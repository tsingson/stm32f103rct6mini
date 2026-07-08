#include <string.h>
#include <zephyr/kernel.h>
#include "lte_gps_logic.h"
#include "lte_modem_interface.h"

#define FRAME_HEADER_0       0xAAU
#define FRAME_HEADER_1       0x55U
#define CMD_HEARTBEAT        0x01U
#define CMD_GPS_DATA         0x02U
#define CMD_NTP_REQUEST      0x03U
#define CMD_NTP_RESPONSE     0x04U

#define SYSTEM_BASE_TIMESTAMP 1767225600UL
#define LTE_RX_BUF_SIZE       1024
#define LTE_SERVER_HOST       "142.54.180.58"
#define LTE_SERVER_PORT       8061U
#define LTE_SEND_RETRY_COUNT   3U
#define LTE_ACK_TIMEOUT_MS     2500U
#define LTE_NTP_TIMEOUT_MS     3500U
#define LTE_RETRY_BASE_MS      120U

static uint32_t system_timestamp = SYSTEM_BASE_TIMESTAMP;

uint32_t get_system_timestamp(void)
{
    uint32_t uptime_sec = k_uptime_get_32() / 1000U;
    return system_timestamp + uptime_sec;
}

void set_system_timestamp(uint32_t ts)
{
    system_timestamp = ts;
}

heartbeat_report_t* heartbeat_create(uint8_t status, uint8_t battery,
                                     uint16_t sleep_time, const char* dev_id)
{
    if (dev_id == NULL) {
        return NULL;
    }

    heartbeat_report_t* hb = k_malloc(sizeof(heartbeat_report_t));
    if (hb == NULL) {
        return NULL;
    }

    hb->status = status;
    hb->battery = battery;
    hb->sleep_countdown = sleep_time;
    strncpy(hb->device_id, dev_id, sizeof(hb->device_id) - 1U);
    hb->device_id[sizeof(hb->device_id) - 1U] = '\0';

    return hb;
}

void heartbeat_destroy(heartbeat_report_t* hb)
{
    if (hb != NULL) {
        k_free(hb);
    }
}

bool heartbeat_serialize(const heartbeat_report_t* hb, uint8_t* buffer, size_t* out_len)
{
    if (hb == NULL || buffer == NULL || out_len == NULL) {
        return false;
    }

    size_t id_len = strlen(hb->device_id);
    size_t total_len = 4U + id_len;

    if (total_len > 256U) {
        return false;
    }

    buffer[0] = hb->status;
    buffer[1] = hb->battery;
    buffer[2] = (hb->sleep_countdown >> 8) & 0xFFU;
    buffer[3] = hb->sleep_countdown & 0xFFU;
    memcpy(&buffer[4], hb->device_id, id_len);

    *out_len = total_len;
    return true;
}

static uint32_t retry_backoff_ms(uint32_t attempt)
{
    uint32_t delay = LTE_RETRY_BASE_MS << attempt;
    return (delay > 800U) ? 800U : delay;
}

static bool wait_for_pattern(const uint8_t* pattern, size_t pattern_len,
                             uint32_t timeout_ms, size_t* match_offset)
{
    if (pattern == NULL || pattern_len == 0U) {
        return false;
    }

    uint8_t snapshot[LTE_RX_BUF_SIZE];
    uint32_t start = k_uptime_get_32();
    uint32_t sleep_ms = 10U;

    while ((k_uptime_get_32() - start) < timeout_ms) {
        size_t rx_len = lte_snapshot_rx_raw(snapshot, sizeof(snapshot));
        if (rx_len >= pattern_len) {
            for (size_t i = 0U; i + pattern_len <= rx_len; i++) {
                if (memcmp(&snapshot[i], pattern, pattern_len) == 0) {
                    if (match_offset != NULL) {
                        *match_offset = i;
                    }
                    return true;
                }
            }
        }

        k_sleep(K_MSEC(sleep_ms));
        if (sleep_ms < 80U) {
            sleep_ms <<= 1;
        }
    }

    return false;
}

bool ntp_sync(uint32_t* out_timestamp)
{
    if (modem == NULL || out_timestamp == NULL) {
        return false;
    }

    uint8_t ntp_req[4] = { FRAME_HEADER_0, FRAME_HEADER_1, CMD_NTP_REQUEST, 0U };

    for (uint32_t attempt = 0U; attempt < LTE_SEND_RETRY_COUNT; attempt++) {
        lte_clear_rx_buffer();

        if (!modem->socket_open(0, LTE_SERVER_HOST, LTE_SERVER_PORT)) {
            k_sleep(K_MSEC(retry_backoff_ms(attempt)));
            continue;
        }

        if (!modem->socket_send_prepare(0, sizeof(ntp_req))) {
            modem->socket_close(0);
            k_sleep(K_MSEC(retry_backoff_ms(attempt)));
            continue;
        }

        lte_clear_rx_buffer();
        if (!lte_send_raw_stream(ntp_req, sizeof(ntp_req))) {
            modem->socket_close(0);
            k_sleep(K_MSEC(retry_backoff_ms(attempt)));
            continue;
        }

        const uint8_t ntp_prefix[4] = { FRAME_HEADER_0, FRAME_HEADER_1, CMD_NTP_RESPONSE, 4U };
        size_t match_offset = 0U;

        if (wait_for_pattern(ntp_prefix, sizeof(ntp_prefix), LTE_NTP_TIMEOUT_MS, &match_offset)) {
            uint8_t snapshot[LTE_RX_BUF_SIZE];
            size_t rx_len = lte_snapshot_rx_raw(snapshot, sizeof(snapshot));
            if (rx_len >= match_offset + 8U) {
                uint32_t ntp_time = ((uint32_t)snapshot[match_offset + 4U] << 24) |
                                    ((uint32_t)snapshot[match_offset + 5U] << 16) |
                                    ((uint32_t)snapshot[match_offset + 6U] << 8) |
                                    ((uint32_t)snapshot[match_offset + 7U]);

                system_timestamp = ntp_time;
                *out_timestamp = ntp_time;
                modem->socket_close(0);
                return true;
            }
        }

        modem->socket_close(0);
        k_sleep(K_MSEC(retry_backoff_ms(attempt)));
    }

    return false;
}

static bool tcp_send_frame(uint8_t cmd, const uint8_t* payload, size_t payload_len,
                           const char* server_host, uint16_t server_port)
{
    if (modem == NULL || payload == NULL || server_host == NULL) {
        return false;
    }

    if (payload_len > UINT8_MAX) {
        return false;
    }

    uint8_t packet[4U + 256U];
    size_t packet_len = 4U + payload_len;

    packet[0] = FRAME_HEADER_0;
    packet[1] = FRAME_HEADER_1;
    packet[2] = cmd;
    packet[3] = (uint8_t)payload_len;
    memcpy(&packet[4], payload, payload_len);

    const uint8_t ack = 0x01U;

    for (uint32_t attempt = 0U; attempt < LTE_SEND_RETRY_COUNT; attempt++) {
        lte_clear_rx_buffer();

        if (!modem->socket_open(0, server_host, server_port)) {
            k_sleep(K_MSEC(retry_backoff_ms(attempt)));
            continue;
        }

        if (!modem->socket_send_prepare(0, packet_len)) {
            modem->socket_close(0);
            k_sleep(K_MSEC(retry_backoff_ms(attempt)));
            continue;
        }

        lte_clear_rx_buffer();
        if (!lte_send_raw_stream(packet, packet_len)) {
            modem->socket_close(0);
            k_sleep(K_MSEC(retry_backoff_ms(attempt)));
            continue;
        }

        if (wait_for_pattern(&ack, 1U, LTE_ACK_TIMEOUT_MS, NULL)) {
            modem->socket_close(0);
            return true;
        }

        modem->socket_close(0);
        k_sleep(K_MSEC(retry_backoff_ms(attempt)));
    }

    return false;
}

bool gps_report_send(const gps_report_t* report, const char* server_host, uint16_t server_port)
{
    if (report == NULL) {
        return false;
    }

    uint8_t payload[256];
    size_t payload_len = 0U;

    if (!gps_report_serialize(report, payload, &payload_len)) {
        return false;
    }

    return tcp_send_frame(CMD_GPS_DATA, payload, payload_len, server_host, server_port);
}

bool heartbeat_send(const heartbeat_report_t* hb, const char* server_host, uint16_t server_port)
{
    if (hb == NULL) {
        return false;
    }

    uint8_t payload[256];
    size_t payload_len = 0U;

    if (!heartbeat_serialize(hb, payload, &payload_len)) {
        return false;
    }

    return tcp_send_frame(CMD_HEARTBEAT, payload, payload_len, server_host, server_port);
}
