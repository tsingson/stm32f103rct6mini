#ifndef LTE_MODEM_INTERFACE_H_
#define LTE_MODEM_INTERFACE_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* 定义4G模组通用接口结构体 */
struct lte_modem_driver {
    const char* name;
    bool (*init_modem)(void);
    bool (*check_alive)(void);
    bool (*connect_network)(const char* apn);
    bool (*socket_open)(int sock_id, const char* host, uint16_t port);
    bool (*socket_send_prepare)(int sock_id, size_t data_len);
    bool (*socket_close)(int sock_id);
    const char* (*get_disconn_indicator)(void);
};

extern const struct lte_modem_driver* modem;
extern volatile size_t lte_rx_idx;

void lte_clear_rx_buffer(void);
size_t lte_snapshot_rx_raw(uint8_t* dst, size_t dst_len);
void lte_snapshot_rx(char* dst, size_t dst_len);
bool lte_send_at_cmd(const char* cmd);
bool lte_wait_for(const char* expected, uint32_t timeout_ms);
bool lte_send_cmd_expect(const char* cmd, const char* expected, uint32_t timeout_ms);
bool lte_send_raw_stream(const uint8_t* data, size_t len);
void lte_4g_init(void);

#endif /* LTE_MODEM_INTERFACE_H_ */
