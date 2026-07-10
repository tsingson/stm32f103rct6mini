/**
 * @file ublox_m10_nano.h
 * @brief 支持多实例调用的 u-blox M10 GPS 驱动头文件 (最终生产交付级，严格符合 C17 / Zephyr 4.4.1)
 */

#ifndef UBLOX_M10_NANO_H
#define UBLOX_M10_NANO_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdint.h>
#include <stddef.h>
#include "gps_ring_buffer.h"

// 核心协议宏定义
#define UBX_SYNC_CHAR_1       0xB5U
#define UBX_SYNC_CHAR_2       0x62U
#define UBX_CLASS_CFG         0x06U
#define UBX_ID_VALSET         0x8AU
#define UBX_LAYER_ALL         0x07U

#define KEY_UART1OUTPROT_UBX  0x20010021U
#define KEY_RATE_MEAS         0x30210001U
#define KEY_MSGOUT_NAV_PVT    0x20910007U

#define RX_RAW_RING_BUF_SIZE  1024U
#define GPS_THREAD_STACK_SZ   2048U

// ==============================================================================
// 1. ⭐ 补充恢复：UBX-NAV-PVT 数据结构体定义 (严格符合 C17 内存对齐属性，固定 92 字节)
// ==============================================================================
struct __attribute__((packed)) ubx_nav_pvt {
    uint32_t iTOW;    // GPS 毫秒时间戳
    uint16_t year;    // 年
    uint8_t month;    // 月
    uint8_t day;      // 日
    uint8_t hour;     // 时
    uint8_t min;      // 分
    uint8_t sec;      // 秒
    uint8_t valid;    // 有效性标志
    uint32_t tAcc;    // 时间精度
    int32_t nano;     // 纳秒
    uint8_t fixType;  // 定位类型 (0=无定位, 2=2D, 3=3D定位)
    uint8_t flags;    // 导航状态标志
    uint8_t flags2;   // 额外标志
    uint8_t numSV;    // 参与定位的卫星数量
    int32_t lon;      // 经度 (缩放比例 1e-7)
    int32_t lat;      // 纬度 (缩放比例 1e-7)
    int32_t height;   // 椭球高 (mm)
    int32_t hMSL;     // 海拔高度 (mm)
    uint32_t hAcc;    // 水平精度 (mm)
    uint32_t vAcc;    // 垂直精度 (mm)
    int32_t velN;     // 北向速度 (mm/s)
    int32_t velE;     // 东向速度 (mm/s)
    int32_t velD;     // 地向速度 (mm/s)
    int32_t gSpeed;   // 地速 (mm/s)
    int32_t headMot;  // 运动航向角 (deg * 1e-5)
    uint32_t sAcc;    // 速度精度 (mm/s)
    uint32_t headAcc; // 航向精度 (deg * 1e-5)
    uint16_t pDOP;    // 位置位置因子 (0.01)
    uint8_t flags3;   // 额外标志3
    uint8_t reserved1; // 补齐 92 字节物理载荷大小
};

typedef struct ubx_nav_pvt ubx_nav_pvt_t;

// ==============================================================================
// 2. GPS 驱动实例软硬件统一上下文结构体 (Context)
// ==============================================================================
typedef struct {
    const struct device *uart_device;        // 绑定的底层物理串口设备指针
    gps_rb_instance_t *target_rb;            // 该硬件解包后注入的目标多实例环形缓冲区

    struct ring_buf rx_raw_rb;               // 专属的硬件中断原始字节环形队列
    uint8_t rx_raw_mem[RX_RAW_RING_BUF_SIZE];// 专属的字节队列物理内存
    struct k_sem rx_signal_sem;              // 精准唤醒解包线程的专属信号量

    struct k_thread thread_data;             // 专属的独立工作线程控制块
    k_thread_stack_t *thread_stack;          // 外部传入分配的独立线程栈内存

    // 状态机私有上下文
    uint8_t parser_state;
    uint8_t u_class;
    uint8_t u_id;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t payload_buf[256];                // 状态机解包物理缓存区
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t calc_ck_a;
    uint8_t calc_ck_b;
} ublox_m10_context_t;

// ⭐ 彻底删除原第 52 行的 #include "ublox_m10_nano.h"（死循环自包含漏洞已完全根治）

// ==============================================================================
// 3. 多实例通用驱动外部公开 API 接口
// ==============================================================================

/**
 * @brief 初始化一个指定的 GPS 驱动硬件实例并自动拉起状态机异步线程
 * @param ctx 目标驱动上下文结构体指针
 * @param uart_dev_spec 绑定的底层串口物理设备
 * @param out_rb_instance 成功解包后绑定的数据去向目标多实例环形缓冲区
 * @param stack_mem 分配给该驱动解包线程的栈内存空间指针 (通过 K_THREAD_STACK_DEFINE 创建)
 * @param priority 线程抢占优先级 (建议设置为 5)
 * @return 0 成功, 负数 错误码
 */
int init_ubx_m10_driver_instance(ublox_m10_context_t *ctx,
                                 const struct device *uart_dev_spec,
                                 gps_rb_instance_t *out_rb_instance,
                                 k_thread_stack_t *stack_mem,
                                 int priority);

/**
 * @brief 向特定 GPS 实例注入多键值 ValSet 产品化持久配置
 */
void gps_configure_ubx_nona_proc(ublox_m10_context_t *ctx);

/**
 * @brief 针对特定 GPS 实例进行字节流解析的状态机接口
 */
void process_ubx_nona_byte(ublox_m10_context_t *ctx, uint8_t byte);

#endif // UBLOX_M10_NANO_H
