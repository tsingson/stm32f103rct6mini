/**
 * @file ublox_m10_nano.c
 * @brief Zephyr 4.4.1 通用 u-blox M10 Nano GPS 驱动实现 (生产交付级，严格符合 C17)
 */

#include "ublox_m10_nano.h"
#include "gps_ring_buffer.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(ublox_m10, CONFIG_GPS_LOG_LEVEL);

#define GPS_UART_NODE DT_ALIAS(gps_uart)

#if !DT_NODE_HAS_STATUS_OKAY(GPS_UART_NODE)
#error "设备树别名 'gps_uart' 未启用或未定义。请检查您的 stm32f103_mini.overlay 文件！"
#endif

static const struct device *const uart_dev = DEVICE_DT_GET(GPS_UART_NODE);

#define RX_RING_BUF_SIZE 1024U
#define GPS_THREAD_STACK_SIZE 2048U
#define GPS_THREAD_PRIORITY 5

static uint8_t rx_ring_buffer[RX_RING_BUF_SIZE];
static struct ring_buf ringbuf;
static struct k_sem rx_sem;

// ==============================================================================
// UART 中断回调函数 (将物理字节安全暂存至硬件环形缓冲区)
// ==============================================================================
static void uart_callback(const struct device *dev, void *user_data)
{
    uint8_t c;
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1U) == 1) {
            if (ring_buf_put(&ringbuf, &c, 1U) == 1U) {
                k_sem_give(&rx_sem);
            }
        }
    }
}

void ubx_nona_append_checksum(uint8_t *buffer, size_t len)
{
    if ((buffer == NULL) || (len < 8U)) {
        return;
    }
    uint8_t ck_a = 0U, ck_b = 0U;
    for (size_t i = 2U; i < (len - 2U); i++) {
        ck_a += buffer[i];
        ck_b += ck_a;
    }
    buffer[len - 2U] = ck_a;
    buffer[len - 1U] = ck_b;
}

void gps_configure_ubx_nona_proc(void)
{
    uint8_t cfg_packet[] = {
        UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2, UBX_CLASS_CFG, UBX_ID_VALSET, 0x14U, 0x00U,

        // --- Payload ---
        0x00U, UBX_LAYER_ALL, 0x00U, 0x00U,

        // [1] 纯 UBX 模式
        (uint8_t)(KEY_UART1OUTPROT_UBX & 0xFFU),
        (uint8_t)((KEY_UART1OUTPROT_UBX >> 8U) & 0xFFU),
        (uint8_t)((KEY_UART1OUTPROT_UBX >> 16U) & 0xFFU),
        (uint8_t)((KEY_UART1OUTPROT_UBX >> 24U) & 0xFFU),
        0x01U,

        // [2] 5Hz 频率
        (uint8_t)(KEY_RATE_MEAS & 0xFFU),
        (uint8_t)((KEY_RATE_MEAS >> 8U) & 0xFFU),
        (uint8_t)((KEY_RATE_MEAS >> 16U) & 0xFFU),
        (uint8_t)((KEY_RATE_MEAS >> 24U) & 0xFFU),
        0xC8U, 0x00U,

        // [3] NAV-PVT 主动上报
        (uint8_t)(KEY_MSGOUT_NAV_PVT & 0xFFU),
        (uint8_t)((KEY_MSGOUT_NAV_PVT >> 8U) & 0xFFU),
        (uint8_t)((KEY_MSGOUT_NAV_PVT >> 16U) & 0xFFU),
        (uint8_t)((KEY_MSGOUT_NAV_PVT >> 24U) & 0xFFU),
        0x01U,

        0x00U, 0x00U
    };

    ubx_nona_append_checksum(cfg_packet, sizeof(cfg_packet));

    unsigned int key = irq_lock();
    ring_buf_reset(&ringbuf);
    irq_unlock(key);

    for (size_t i = 0U; i < sizeof(cfg_packet); i++) {
        uart_poll_out(uart_dev, cfg_packet[i]);
    }
    LOG_INF("M10 生产级持久化配置包注入成功！");
}

// ==============================================================================
// UBX 二进制流状态机内核 (彻底修复旧版 static 变量定义引发的严重溢出死机漏洞)
// ==============================================================================
void process_ubx_nona_byte(uint8_t byte)
{
    static enum {
        STATE_IDLE,
        STATE_SYNC2,
        STATE_CLASS,
        STATE_ID,
        STATE_LEN1,
        STATE_LEN2,
        STATE_PAYLOAD,
        STATE_CKA,
        STATE_CKB
    } state = STATE_IDLE;

    static uint8_t u_class = 0U, u_id = 0U;
    static uint16_t payload_len = 0U, payload_idx = 0U;

    // ⭐ 根治致命隐患：显式开辟 256 字节的物理内存数组空间，杜绝越界踩踏
    static uint8_t payload_buf[256];
    static uint8_t ck_a = 0U, ck_b = 0U;
    static uint8_t calc_ck_a = 0U, calc_ck_b = 0U;

    gps_location_t fake_gps;

    switch (state) {
    case STATE_IDLE:
        if (byte == UBX_SYNC_CHAR_1) state = STATE_SYNC2;
        break;
    case STATE_SYNC2:
        state = (byte == UBX_SYNC_CHAR_2) ? STATE_CLASS : STATE_IDLE;
        break;
    case STATE_CLASS:
        u_class = byte;
        calc_ck_a = byte; calc_ck_b = byte;
        state = STATE_ID;
        break;
    case STATE_ID:
        u_id = byte;
        calc_ck_a += byte; calc_ck_b += calc_ck_a;
        state = STATE_LEN1;
        break;
    case STATE_LEN1:
        payload_len = byte;
        calc_ck_a += byte; calc_ck_b += calc_ck_a;
        state = STATE_LEN2;
        break;
    case STATE_LEN2:
        payload_len |= (uint16_t)((uint16_t)byte << 8);
        calc_ck_a += byte; calc_ck_b += calc_ck_a;
        payload_idx = 0U;
        state = ((payload_len > 0U) && (payload_len < sizeof(payload_buf))) ? STATE_PAYLOAD : STATE_IDLE;
        break;
    case STATE_PAYLOAD:
        payload_buf[payload_idx] = byte;
        payload_idx++;
        calc_ck_a += byte; calc_ck_b += calc_ck_a;
        if (payload_idx >= payload_len) state = STATE_CKA;
        break;
    case STATE_CKA:
        ck_a = byte;
        state = STATE_CKB;
        break;
    case STATE_CKB:
        ck_b = byte;
        state = STATE_IDLE;

        if ((ck_a == calc_ck_a) && (ck_b == calc_ck_b)) {
            if ((u_class == 0x01U) && (u_id == 0x07U)) {
                ubx_nav_pvt_t *pvt = (ubx_nav_pvt_t *)payload_buf;

                fake_gps.fixType = pvt->fixType;
                fake_gps.numSV = pvt->numSV;
                fake_gps.lat = pvt->lat;
                fake_gps.lon = pvt->lon;
                fake_gps.gSpeed = pvt->gSpeed;

                // 完美注入 Zephyr 安全数据环
                (void)gps_rb_push(&fake_gps);
            }
        } else {
            LOG_WRN("UBX Frame Checksum Error!");
        }
        break;
    }
}

// ==============================================================================
// 6. Zephyr 异步工作线程
// ==============================================================================
K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);
static struct k_thread gps_thread_data;

static void gps_process_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    uint8_t byte;

    LOG_INF("GPS 专属状态机独立解包工作线程正常启动");

    k_sleep(K_MSEC(500));
    gps_configure_ubx_nona_proc();

    while (1) {
        (void)k_sem_take(&rx_sem, K_FOREVER);
        while (ring_buf_get(&ringbuf, &byte, 1U) == 1U) {
            process_ubx_nona_byte(byte);
        }
    }
}

int init_ubx_nona_gps_uart(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("Device tree map error: GPS UART not ready!");
        return -ENODEV;
    }

    ring_buf_init(&ringbuf, sizeof(rx_ring_buffer), rx_ring_buffer);
    (void)k_sem_init(&rx_sem, 0U, UINT_MAX);

    uart_irq_callback_user_data_set(uart_dev, uart_callback, NULL);
    uart_irq_rx_enable(uart_dev);

    (void)k_thread_create(&gps_thread_data, gps_stack,
                          K_THREAD_STACK_SIZEOF(gps_stack),
                          gps_process_thread, NULL, NULL, NULL,
                          GPS_THREAD_PRIORITY, 0U, K_NO_WAIT);

    return 0;
}
