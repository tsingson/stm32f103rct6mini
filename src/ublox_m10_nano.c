/**
 * @file ublox_m10_nano.c
 * @brief Zephyr 4.4.1 通用 u-blox M10 Nano GPS 驱动实现 (100% 对齐原版解包逻辑)
 */

#include "ublox_m10_nano.h"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ublox_m10, CONFIG_GPS_LOG_LEVEL);

// 定义 100% 对齐原版的状态机内部枚举
enum
{
    STATE_IDLE,
    STATE_SYNC2,
    STATE_CLASS,
    STATE_ID,
    STATE_LEN1,
    STATE_LEN2,
    STATE_PAYLOAD,
    STATE_CKA,
    STATE_CKB
};

// ==============================================================================
// 1. UART 中断服务函数 (无缝对接特定实例的 Context，支持多实例并发现收)
// ==============================================================================
static void uart_callback(const struct device* dev, void* user_data)
{
    uint8_t c;
    ublox_m10_context_t* ctx = (ublox_m10_context_t*)user_data;

    if ((ctx == NULL) || !uart_irq_update(dev))
    {
        return;
    }

    if (uart_irq_rx_ready(dev))
    {
        while (uart_fifo_read(dev, &c, 1U) == 1)
        {
            // 压入该物理实例所属的专属接收缓冲区
            if (ring_buf_put(&ctx->rx_raw_rb, &c, 1U) == 1U)
            {
                k_sem_give(&ctx->rx_signal_sem); // 精准唤醒该硬件对应的专用解包任务
            }
        }
    }
}

void ubx_nona_append_checksum(uint8_t* buffer, size_t len)
{
    if ((buffer == NULL) || (len < 8U))
    {
        return;
    }
    uint8_t ck_a = 0U, ck_b = 0U;
    // Fletcher 算法：从 Class 字节开始，累加到 Checksum 之前
    for (size_t i = 2U; i < (len - 2U); i++)
    {
        ck_a += buffer[i];
        ck_b += ck_a;
    }
    buffer[len - 2U] = ck_a;
    buffer[len - 1U] = ck_b;
}

void gps_configure_ubx_nona_proc(ublox_m10_context_t* ctx)
{
    if ((ctx == NULL) || (ctx->uart_device == NULL))
    {
        return;
    }

    uint8_t cfg_packet[] = {
        UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2, UBX_CLASS_CFG, UBX_ID_VALSET, 0x14U, 0x00U,

        // --- Payload 开始 ---
        0x00U, // Version: 0
        UBX_LAYER_ALL, // ⭐ 全层持久化写入（RAM + BBR + Flash），断电不失忆
        0x00U, 0x00U, // 保留位占位

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

        0x00U, 0x00U // Checksum 占位 (CK_A, CK_B)
    };

    ubx_nona_append_checksum(cfg_packet, sizeof(cfg_packet));

    // 发送前清空输入缓冲区，防止残留 NMEA 文本污染后续解析
    unsigned int key = irq_lock();
    ring_buf_reset(&ctx->rx_raw_rb);
    irq_unlock(key);

    for (size_t i = 0U; i < sizeof(cfg_packet); i++)
    {
        uart_poll_out(ctx->uart_device, cfg_packet[i]);
    }
    LOG_INF("M10 生产级持久化配置包已全量安全注入！");
}

// ==============================================================================
// 2. 多实例解耦下的 UBX 二进制流状态机内核 (100% 像素级对齐您的原版运行逻辑)
// ==============================================================================
void process_ubx_nona_byte(ublox_m10_context_t* ctx, uint8_t byte)
{
    if ((ctx == NULL) || (ctx->target_rb == NULL))
    {
        return;
    }
    gps_location_t fake_gps;

    switch (ctx->parser_state)
    {
    case STATE_IDLE:
        if (byte == UBX_SYNC_CHAR_1)
        {
            ctx->parser_state = STATE_SYNC2;
        }
        break;
    case STATE_SYNC2:
        ctx->parser_state = (byte == UBX_SYNC_CHAR_2) ? STATE_CLASS : STATE_IDLE;
        break;
    case STATE_CLASS:
        ctx->u_class = byte;
        ctx->calc_ck_a = byte;
        ctx->calc_ck_b = byte; // 复位 Fletcher 校验
        ctx->parser_state = STATE_ID;
        break;
    case STATE_ID:
        ctx->u_id = byte;
        ctx->calc_ck_a += byte;
        ctx->calc_ck_b += ctx->calc_ck_a;
        ctx->parser_state = STATE_LEN1;
        break;
    case STATE_LEN1:
        ctx->payload_len = byte;
        ctx->calc_ck_a += byte;
        ctx->calc_ck_b += ctx->calc_ck_a;
        ctx->parser_state = STATE_LEN2;
        break;
    case STATE_LEN2:
        ctx->payload_len |= (uint16_t)((uint16_t)byte << 8);
        ctx->calc_ck_a += byte;
        ctx->calc_ck_b += ctx->calc_ck_a;
        ctx->payload_idx = 0U;
        // 长度防御性限制，防止恶意长数据包撑爆本地 RAM 缓冲区
        ctx->parser_state = ((ctx->payload_len > 0U) && (ctx->payload_len < sizeof(ctx->payload_buf)))
                                ? STATE_PAYLOAD
                                : STATE_IDLE;
        break;
    case STATE_PAYLOAD:
        ctx->payload_buf[ctx->payload_idx++] = byte; // 100% 还原原版自增语序
        ctx->calc_ck_a += byte;
        ctx->calc_ck_b += ctx->calc_ck_a;
        if (ctx->payload_idx >= ctx->payload_len)
        {
            ctx->parser_state = STATE_CKA;
        }
        break;
    case STATE_CKA:
        ctx->ck_a = byte;
        ctx->parser_state = STATE_CKB;
        break;
    case STATE_CKB:
        ctx->ck_b = byte;
        ctx->parser_state = STATE_IDLE; // 本帧结束，状态机复位

        // 严苛的端到端数据校验
        if ((ctx->ck_a == ctx->calc_ck_a) && (ctx->ck_b == ctx->calc_ck_b))
        {
            // 成功捕获高频综合导航包 (Class: 0x01, ID: 0x07 -> UBX-NAV-PVT)
            if ((ctx->u_class == 0x01U) && (ctx->u_id == 0x07U))
            {
                // 100% 还原原版内存直接映射逻辑，调用在头文件中严格定义的 92 字节标准型
                ubx_nav_pvt_t* pvt = (ubx_nav_pvt_t*)ctx->payload_buf;

                fake_gps.fixType = pvt->fixType;
                fake_gps.numSV = pvt->numSV;
                fake_gps.lat = pvt->lat;
                fake_gps.lon = pvt->lon;
                fake_gps.gSpeed = pvt->gSpeed;

                // 完美推入多实例指定的、经过防系统崩溃优化的 Zephyr 原生数据环
                (void)gps_rb_push(ctx->target_rb, &fake_gps);
            }
        }
        else
        {
            LOG_WRN("UBX Frame Checksum Error!");
        }
        break;
    }
}

// ==============================================================================
// 3. 多实例通用独立工作线程函数
// ==============================================================================
static void gps_instance_process_thread(void* p1, void* p2, void* p3)
{
    ublox_m10_context_t* ctx = (ublox_m10_context_t*)p1;
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    uint8_t byte;

    if (ctx == NULL)
    {
        return;
    }

    LOG_INF("GPS 驱动实例独立解包工作线程正常拉起.");
    k_sleep(K_MSEC(500)); // 留出时间等待硬件和驱动层完全就绪
    gps_configure_ubx_nona_proc(ctx); // 执行动态产品化配置

    while (1)
    {
        // 无数据挂起，该实例的专属 ISR 塞入字节时被精准唤醒
        (void)k_sem_take(&ctx->rx_signal_sem, K_FOREVER);
        while (ring_buf_get(&ctx->rx_raw_rb, &byte, 1U) == 1U)
        {
            process_ubx_nona_byte(ctx, byte); // 字节流不间断喂给状态机解析
        }
    }
}

// ==============================================================================
// 4. 通用多实例构造与配置入口
// ==============================================================================
int init_ubx_m10_driver_instance(ublox_m10_context_t* ctx,
                                 const struct device* uart_dev_spec,
                                 gps_rb_instance_t* out_rb_instance,
                                 k_thread_stack_t* stack_mem,
                                 int priority)
{
    if ((ctx == NULL) || (uart_dev_spec == NULL) || (out_rb_instance == NULL) || (stack_mem == NULL))
    {
        return -EINVAL;
    }

    if (!device_is_ready(uart_dev_spec))
    {
        LOG_ERR("传入物理串口设备未就绪！");
        return -ENODEV;
    }

    // 上下文成员初赋初值
    ctx->uart_device = uart_dev_spec;
    ctx->target_rb = out_rb_instance;
    ctx->parser_state = STATE_IDLE;

    // 初始化内核同步对象
    ring_buf_init(&ctx->rx_raw_rb, sizeof(ctx->rx_raw_mem), ctx->rx_raw_mem);
    (void)k_sem_init(&ctx->rx_signal_sem, 0U, UINT_MAX);

    // 绑定物理串口中断回调，并将 ctx 上下文指针作为 user_data 参数灌入中断
    uart_irq_callback_user_data_set(ctx->uart_device, uart_callback, (void*)ctx);
    uart_irq_rx_enable(ctx->uart_device);

    // 为该实例动态拉起独立的状态机工作线程
    (void)k_thread_create(&ctx->thread_data, stack_mem,
                          GPS_THREAD_STACK_SZ,
                          gps_instance_process_thread, (void*)ctx, NULL, NULL,
                          priority, 0U, K_NO_WAIT);

    return 0;
}
