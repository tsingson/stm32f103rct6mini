/**
 * @file ublox_m10_nano.h
 * @brief Zephyr 4.4.1 通用 u-blox M10 Nano GPS 驱动头文件 (生产交付级，严格符合 C17)
 */

#ifndef UBLOX_M10_NANO_H
#define UBLOX_M10_NANO_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
#include <stddef.h>

// ==============================================================================
// 1. 核心协议宏定义
// ==============================================================================
#define UBX_SYNC_CHAR_1       0xB5U
#define UBX_SYNC_CHAR_2       0x62U
#define UBX_CLASS_CFG         0x06U
#define UBX_ID_VALSET         0x8AU

// 配置存储目标层 (Layers)
#define UBX_LAYER_ALL         0x07U

// u-blox M10 配置键值 ID (Key IDs)
#define KEY_UART1OUTPROT_UBX  0x20010021U
#define KEY_RATE_MEAS         0x30210001U
#define KEY_MSGOUT_NAV_PVT    0x20910007U

// ==============================================================================
// 2. UBX-NAV-PVT 数据结构体定义 (严格符合 C17 内存对齐属性，固定 92 字节)
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
    uint8_t reserved1[5]; // 补齐 92 字节物理载荷大小
};

typedef struct ubx_nav_pvt ubx_nav_pvt_t;

// ==============================================================================
// 3. 驱动外部接口 API
// ==============================================================================

/**
 * @brief 初始化 GPS 驱动线程与串口硬件
 * @return 0 成功, 负数 错误码
 */
int init_ubx_nona_gps_uart(void);

/**
 * @brief 计算并追加 UBX 校验和
 */
void ubx_nona_append_checksum(uint8_t *buffer, size_t len);

/**
 * @brief 向 GPS 发送 M10 专属优化配置流
 */
void gps_configure_ubx_nona_proc(void);

/**
 * @brief 字节流状态机解析核心
 */
void process_ubx_nona_byte(uint8_t byte);

#endif // UBLOX_M10_NANO_H
