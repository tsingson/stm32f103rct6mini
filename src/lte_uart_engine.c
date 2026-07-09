#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <string.h>
#include "lte_modem_interface.h"

#define LTE_UART_DEVICE_NODE  DT_ALIAS(lte_uart)
#define LTE_RX_BUF_SIZE       1024

static const struct device* lte_uart_dev;
static uint8_t lte_rx_buffer[LTE_RX_BUF_SIZE];
volatile size_t lte_rx_idx = 0U;

static bool uart_send_bytes(const uint8_t* data, size_t len)
{
    if (data == NULL || len == 0U)
    {
        return false;
    }

    if (!device_is_ready(lte_uart_dev))
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        uart_poll_out(lte_uart_dev, data[i]);
    }

    return true;
}

static void lte_uart_callback(const struct device* dev, void* user_data)
{
    ARG_UNUSED(user_data);
    uint8_t c;

    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev))
    {
        return;
    }

    while (uart_fifo_read(dev, &c, 1) > 0)
    {
        size_t current_idx = lte_rx_idx;
        if (current_idx < LTE_RX_BUF_SIZE)
        {
            lte_rx_buffer[current_idx] = c;
            lte_rx_idx = current_idx + 1U;
            // } else {
            //     lte_rx_buffer[LTE_RX_BUF_SIZE - 1U] = c;
            // }
        }
        else
        {
            /* buffer full — drop incoming byte */
            (void)c;
        }
    }
}

void lte_clear_rx_buffer(void)
{
    unsigned int key = irq_lock();
    lte_rx_idx = 0U;
    memset(lte_rx_buffer, 0, sizeof(lte_rx_buffer));
    irq_unlock(key);
}

size_t lte_snapshot_rx_raw(uint8_t* dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0U)
    {
        return 0U;
    }

    unsigned int key = irq_lock();
    size_t n = lte_rx_idx;
    if (n > LTE_RX_BUF_SIZE)
    {
        n = LTE_RX_BUF_SIZE;
    }

    size_t copy_len = (n < dst_len) ? n : dst_len;
    memcpy(dst, lte_rx_buffer, copy_len);
    irq_unlock(key);

    return copy_len;
}

void lte_snapshot_rx(char* dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0U)
    {
        return;
    }

    size_t copy_len = lte_snapshot_rx_raw((uint8_t*)dst, dst_len - 1U);
    dst[copy_len] = '\0';
}

bool lte_send_at_cmd(const char* cmd)
{
    if (cmd == NULL || !device_is_ready(lte_uart_dev))
    {
        return false;
    }

    if (!uart_send_bytes((const uint8_t*)cmd, strlen(cmd)))
    {
        return false;
    }

    uart_poll_out(lte_uart_dev, '\r');
    uart_poll_out(lte_uart_dev, '\n');
    return true;
}

bool lte_wait_for(const char* expected, uint32_t timeout_ms)
{
    uint32_t start = k_uptime_get_32();
    uint32_t sleep_ms = 10U;
    char snapshot[LTE_RX_BUF_SIZE];

    while ((k_uptime_get_32() - start) < timeout_ms)
    {
        lte_snapshot_rx(snapshot, sizeof(snapshot));

        if (strstr(snapshot, expected) != NULL)
        {
            return true;
        }
        if (strstr(snapshot, "ERROR") != NULL)
        {
            return false;
        }

        k_sleep(K_MSEC(sleep_ms));
        if (sleep_ms < 80U)
        {
            sleep_ms <<= 1;
        }
    }

    return false;
}

bool lte_send_cmd_expect(const char* cmd, const char* expected, uint32_t timeout_ms)
{
    lte_clear_rx_buffer();

    if (!lte_send_at_cmd(cmd))
    {
        return false;
    }

    return lte_wait_for(expected, timeout_ms);
}

bool lte_send_raw_stream(const uint8_t* data, size_t len)
{
    if (data == NULL || len == 0U || !device_is_ready(lte_uart_dev))
    {
        return false;
    }

    return uart_send_bytes(data, len);
}

void lte_4g_init(void)
{
    lte_uart_dev = DEVICE_DT_GET(LTE_UART_DEVICE_NODE);
    if (!device_is_ready(lte_uart_dev))
    {
        return;
    }

    uart_irq_callback_user_data_set(lte_uart_dev, lte_uart_callback, NULL);
    uart_irq_rx_enable(lte_uart_dev);
}
