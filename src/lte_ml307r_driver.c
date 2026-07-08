#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "lte_modem_interface.h"

#define LTE_MAX_RETRY_COUNT  5
#define MAX_HOST_LEN         48
#define VALID_SOCKET_ID(id)  ((id) >= 0 && (id) <= 5)

static bool ml307r_init_modem(void)
{
    if (!lte_send_cmd_expect("AT", "OK", 1000)) return false;
    (void)lte_send_cmd_expect("ATE0;+CMEE=2", "OK", 1000);
    if (!lte_send_cmd_expect("AT+CPIN?", "READY", 2000)) return false;
    return true;
}

static bool ml307r_check_alive(void)
{
    return lte_send_cmd_expect("AT+CSQ", "OK", 1500);
}

static bool ml307r_connect_network(const char* apn)
{
    if (apn == NULL || strlen(apn) == 0) return false;
    
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", apn);
    if (!lte_send_cmd_expect(cmd, "OK", 3000)) return false;

    for (int i = 0; i < LTE_MAX_RETRY_COUNT; i++)
    {
        if (lte_send_cmd_expect("AT+CGACT=1,1", "OK", 5000))
        {
            return true;
        }
        k_sleep(K_MSEC(2000));
    }
    return false;
}

static bool ml307r_socket_open(int sock_id, const char* host, uint16_t port)
{
    if (!VALID_SOCKET_ID(sock_id)) return false;
    if (host == NULL || strlen(host) >= MAX_HOST_LEN) return false;
    
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d", sock_id);
    (void)lte_send_cmd_expect(cmd, "OK", 1000);

    snprintf(cmd, sizeof(cmd), "AT+MIPOPEN=%d,\"TCP\",\"%s\",%u", sock_id, host, port);
    if (!lte_send_cmd_expect(cmd, "OK", 10000)) return false;

    k_sleep(K_MSEC(2000));
    return true;
}

static bool ml307r_socket_send_prepare(int sock_id, size_t data_len)
{
    if (!VALID_SOCKET_ID(sock_id)) return false;
    if (data_len == 0 || data_len > 65535) return false;
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+MIPSEND=%d,%zu", sock_id, data_len);
    lte_clear_rx_buffer();
    lte_send_at_cmd(cmd);
    return lte_wait_for(">", 3000);
}

static bool ml307r_socket_close(int sock_id)
{
    if (!VALID_SOCKET_ID(sock_id)) return false;
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+MIPCLOSE=%d", sock_id);
    return lte_send_cmd_expect(cmd, "OK", 2000);
}

static const char* ml307r_get_disconn_indicator(void)
{
    return "disconn";
}

static const struct lte_modem_driver ml307r_driver = {
    .name = "ML307R-DL",
    .init_modem = ml307r_init_modem,
    .check_alive = ml307r_check_alive,
    .connect_network = ml307r_connect_network,
    .socket_open = ml307r_socket_open,
    .socket_send_prepare = ml307r_socket_send_prepare,
    .socket_close = ml307r_socket_close,
    .get_disconn_indicator = ml307r_get_disconn_indicator
};

const struct lte_modem_driver* modem = &ml307r_driver;
