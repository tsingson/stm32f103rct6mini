/**
 * @file ublox_m10_nano.c
 * @brief 严格对齐 FreeRTOS 原版解析逻辑的 Zephyr 4.4.1 通用驱动 (C17 规范)
 */

#include "ublox_m10_nano.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(ublox_m10, CONFIG_GPS_LOG_LEVEL);

// ==============================================================================
// 1. 设备树硬件绑定与内核对象
// ==============================================================================
#define GPS_UART_NODE DT_ALIAS(gps_uart)

#if !DT_NODE_HAS_STATUS_OKAY(GPS_UART_NODE)
#error "设备树别名 'gps_uart' 未启用或未定义。请在您的 app.overlay 中指定！"
#endif

static const struct device *const uart_dev = DEVICE_DT_GET(GPS_UART_NODE);

#define RX_RING_BUF_SIZE 1024U
#define GPS_THREAD_STACK_SIZE 2048U
#define GPS_THREAD_PRIORITY 5

static uint8_t rx_ring_buffer[RX_RING_BUF_SIZE];
static struct ring_buf ringbuf;
static struct k_mutex data_mutex;
static struct k_sem rx_sem;

// 全局最新定位数据副本（供其他线程并发安全调用）
static ubx_nav_pvt_t latest_pvt_data;

// ==============================================================================
// 2. UART 中断服务回调 (ISR 级接收，无缝对接 Zephyr 环形缓冲区)
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

// ==============================================================================
// 3. 基础辅助函数 (100% 对齐原版 Fletcher 算法)
// ==============================================================================
void ubx_nona_append_checksum(uint8_t *buffer, size_t len)
{
    if (len < 8U) {
        return;
    }
    uint8_t ck_a = 0U, ck_b = 0U;
    // Fletcher 算法：从 Class 字节开始，累加到 Checksum 之前
    for (size_t i = 2U; i < (len - 2U); i++) {
        ck_a += buffer[i];
        ck_b += ck_a;
    }
    buffer[len - 2U] = ck_a;
    buffer[len - 1U] = ck_b;
}

// ==============================================================================
// 4. 持久化配置注入 (100% 对齐原版单包多规注入及 Flush 逻辑)
// ==============================================================================
void gps_configure_ubx_nona_proc(void)
{
    uint8_t cfg_packet[] = {
        UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2, UBX_CLASS_CFG, UBX_ID_VALSET, 0x14U, 0x00U, // Payload 长度: 20 字节

        // --- Payload 开始 ---
        0x00U,          // Version: 0
        UBX_LAYER_ALL, // 全层持久化写入（RAM + BBR + Flash）
        0x00U, 0x00U,   // 保留位

        // [项 1] 禁 NMEA 文本，强制转为纯 UBX 二进制流模式
        (uint8_t)(KEY_UART1OUTPROT_UBX & 0xFFU),
        (uint8_t)((KEY_UART1OUTPROT_UBX >> 8U) & 0xFFU),
        (uint8_t)((KEY_UART1OUTPROT_UBX >> 16U) & 0xFFU),
        (uint8_t)((KEY_UART1OUTPROT_UBX >> 24U) & 0xFFU),
        0x01U, // Value: 1 (纯 UBX 模式)

        // [项 2] 飙 5Hz 高频定位 (测量周期 200ms)
        (uint8_t)(KEY_RATE_MEAS & 0xFFU),
        (uint8_t)((KEY_RATE_MEAS >> 8U) & 0xFFU),
        (uint8_t)((KEY_RATE_MEAS >> 16U) & 0xFFU),
        (uint8_t)((KEY_RATE_MEAS >> 24U) & 0xFFU),
        0xC8U, 0x00U, // Value: 200 (U16 小端序)

        // [项 3] 开启 M10 全局 NAV-PVT 消息主动高频推送
        (uint8_t)(KEY_MSGOUT_NAV_PVT & 0xFFU),
        (uint8_t)((KEY_MSGOUT_NAV_PVT >> 8U) & 0xFFU),
        (uint8_t)((KEY_MSGOUT_NAV_PVT >> 16U) & 0xFFU),
        (uint8_t)((KEY_MSGOUT_NAV_PVT >> 24U) & 0xFFU),
        0x01U, // Value: 1 (每次测量输出一次)
        // --- Payload 结束 ---

        0x00U, 0x00U // Checksum 占位
    };

    ubx_nona_append_checksum(cfg_packet, sizeof(cfg_packet));

    // 对齐 uart_flush_input：进入临界区重置 Zephyr 接收环形缓冲区，防历史 NMEA 污染
    unsigned int key = irq_lock();
    ring_buf_reset(&ringbuf);
    irq_unlock(key);

    // 稳定 polling 发送配置包
    for (size_t i = 0U; i < sizeof(cfg_packet); i++) {
        uart_poll_out(uart_dev, cfg_packet[i]);
    }

    LOG_INF("M10 生产级持久化配置包已全量安全注入！");
}

// ==============================================================================
// 5. UBX 二进制流状态机内核 (100% 像素级还原 FreeRTOS 原版状态转换与累加时序)
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

    static uint8_t u_class, u_id;
    static uint16_t payload_len, payload_idx;

    // ⭐ 严格对齐原版：分配 256 字节静态缓冲区，彻底杜绝上版的内存越界隐患
    static uint8_t payload_buf[256];
    static uint8_t ck_a, ck_b;
    static uint8_t calc_ck_a, calc_ck_b;

    switch (state) {
    case STATE_IDLE:
        if (byte == UBX_SYNC_CHAR_1) {
            state = STATE_SYNC2;
        }
        break;
    case STATE_SYNC2:
        state = (byte == UBX_SYNC_CHAR_2) ? STATE_CLASS : STATE_IDLE;
        break;
    case STATE_CLASS:
        u_class = byte;
        calc_ck_a = byte;
        calc_ck_b = byte; // 复位 Fletcher 校验
        state = STATE_ID;
        break;
    case STATE_ID:
        u_id = byte;
        calc_ck_a += byte;
        calc_ck_b += calc_ck_a;
        state = STATE_LEN1;
        break;
    case STATE_LEN1:
        payload_len = byte;
        calc_ck_a += byte;
        calc_ck_b += calc_ck_a;
        state = STATE_LEN2;
        break;
    case STATE_LEN2:
        payload_len |= (uint16_t)((uint16_t)byte << 8U);
        calc_ck_a += byte;
        calc_ck_b += calc_ck_a;
        payload_idx = 0U;
        // 长度防御性限制，防止恶意长数据包撑爆本地 RAM 缓冲区
        state = (payload_len > 0U && payload_len < sizeof(payload_buf)) ? STATE_PAYLOAD : STATE_IDLE;
        break;
    case STATE_PAYLOAD:
        payload_buf[payload_idx++] = byte;
        calc_ck_a += byte;
        calc_ck_b += calc_ck_a;
        if (payload_idx >= payload_len) {
            state = STATE_CKA;
        }
        break;
    case STATE_CKA:
        ck_a = byte;
        state = STATE_CKB;
        break;
    case STATE_CKB:
        ck_b = byte;
        state = STATE_IDLE; // 本帧结束，状态机复位

        // 严苛的端到端数据校验
        if (ck_a == calc_ck_a && ck_b == calc_ck_b) {
            // 成功捕获高频综合导航包 (Class: 0x01, ID: 0x07 -> UBX-NAV-PVT)
            if (u_class == 0x01U && u_id == 0x07U) {

                // 1. 线程安全同步：拷贝至全局区供 Zephyr 其他线程异步读取
                k_mutex_lock(&data_mutex, K_FOREVER);
                memcpy(&latest_pvt_data, payload_buf, sizeof(ubx_nav_pvt_t));
                k_mutex_unlock(&data_mutex);

                // 2. 100% 还原原版内存直接映射逻辑
                ubx_nav_pvt_t *pvt = (ubx_nav_pvt_t *)payload_buf;

                // 3. 规避现代 Zephyr 固件默认不开启 %f 浮点打印的限制，采用安全的整型拆分表达
                int32_t lat_val = pvt->lat;
                int32_t lon_val = pvt->lon;
                int32_t speed_kh_x100 = (int32_t)(((int64_t)pvt->gSpeed * 360LL) / 1000LL);

                printk("[UBX 5Hz 高频解算] 卫星: %d | 定位类型: %d | "
                       "纬度: %d.%07d | 经度: %d.%07d | 地速: %d.%02d km/h\n",
                       pvt->numSV, pvt->fixType,
                       lat_val / 10000000, abs(lat_val % 10000000),
                       lon_val / 10000000, abs(lon_val % 10000000),
                       speed_kh_x100 / 100, abs(speed_kh_x100 % 100));
            }
        } else {
            LOG_WRN("UBX 校验和错误，损坏的帧已安全丢弃");
        }
        break;
    }
}

// ==============================================================================
// 6. Zephyr 异步消费者解包线程 (接管 FreeRTOS 轮询)
// ==============================================================================
K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);
static struct k_thread gps_thread_data;

static void gps_process_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    uint8_t byte;

    LOG_INF("GPS 异步解包内核线程就绪.");

    // 给模块上电、飞控初始化留出充足的硬件电气稳定窗口时间
    k_sleep(K_MSEC(500));
    gps_configure_ubx_nona_proc();

    while (1) {
        // 挂起等待信号量（仅在串口 ISR 接收到新字节时被精准唤醒，不空转 CPU）
        (void)k_sem_take(&rx_sem, K_FOREVER);

        while (ring_buf_get(&ringbuf, &byte, 1U) == 1U) {
            process_ubx_nona_byte(byte);
        }
    }
}

// ==============================================================================
// 7. 驱动通用外部公开 API 接口
// ==============================================================================
int init_ubx_nona_gps_uart(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("设备树映射的 GPS UART 硬件未就绪！");
        return -ENODEV;
    }

    // 初始化 Zephyr 同步内核对象
    ring_buf_init(&ringbuf, sizeof(rx_ring_buffer), rx_ring_buffer);
    (void)k_mutex_init(&data_mutex);
    (void)k_sem_init(&rx_sem, 0U, UINT_MAX);

    // 注册并开启串口中断驱动 API
    uart_irq_callback_user_data_set(uart_dev, uart_callback, NULL);
    uart_irq_rx_enable(uart_dev);

    // 拉起解包任务线程
    (void)k_thread_create(&gps_thread_data, gps_stack,
                          K_THREAD_STACK_SIZEOF(gps_stack),
                          gps_process_thread, NULL, NULL, NULL,
                          GPS_THREAD_PRIORITY, 0U, K_NO_WAIT);

    return 0;
}

void gps_ubx_m10_get_data(ubx_nav_pvt_t *pvt)
{
    if (pvt == NULL) {
        return;
    }
    k_mutex_lock(&data_mutex, K_FOREVER);
    (void)memcpy(pvt, &latest_pvt_data, sizeof(ubx_nav_pvt_t));
    k_mutex_unlock(&data_mutex);
}
