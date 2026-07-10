#ifndef LIS3DH_H
#define LIS3DH_H

#include <zephyr/drivers/spi.h>
#include <stdint.h>
#include <stddef.h>

/* ================================================================= */
/* 💡 LIS3DH 核心寄存器地址映射 (符合 C17 括号隔离保护规范)             */
/* ================================================================= */
#define LIS3DH_REG_WHO_AM_I         (0x0FU)
#define LIS3DH_REG_CTRL_REG1        (0x20U)
#define LIS3DH_REG_CTRL_REG2        (0x21U)
#define LIS3DH_REG_CTRL_REG3        (0x22U)
#define LIS3DH_REG_CTRL_REG4        (0x23U)
#define LIS3DH_REG_OUT_X_L          (0x28U)
#define LIS3DH_REG_INT1_CFG         (0x30U)
#define LIS3DH_REG_INT1_SRC         (0x31U)
#define LIS3DH_REG_INT1_THS         (0x32U)
#define LIS3DH_REG_INT1_DURATION    (0x33U)
#define LIS3DH_REG_REFERENCE        (0x26U) /* 💡 新增：高通滤波器参考/基准寄存器 */

/* ================================================================= */
/* 💡 LIS3DH 全局核心控制 API 声明                                    */
/* ================================================================= */

/**
 * @brief 基础初始化 LIS3DH 芯片并配置默认寄存器
 * @param spi_spec 指向 Zephyr 设备树自动生成的 SPI 配置规范结构体
 * @return int 成功返回 0，失败返回负数内核错误码 (如 -EIO, -EINVAL)
 */
int lis3dh_init(const struct spi_dt_spec *spi_spec);

/**
 * @brief 通用寄存器多字节读取函数
 */
int lis3dh_reg_read(const struct spi_dt_spec *spi_spec, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief 通用单寄存器写入函数
 */
int lis3dh_reg_write(const struct spi_dt_spec *spi_spec, uint8_t reg, uint8_t data);

/**
 * @brief 读取三轴加速度 16 位 ADC 原始寄存器值
 * @param[out] x, y, z 指向接收原始数据的 16 位有符号整型变量
 */
int lis3dh_read_xyz(const struct spi_dt_spec *spi_spec, int16_t *x, int16_t *y, int16_t *z);


/* ================================================================= */
/* 💡 核心新增：超低功耗电源管理及状态机握手 API                         */
/* ================================================================= */

/**
 * @brief 🧠 核心新增：将 LIS3DH 配置为超低功耗 WOM（运动唤醒）中断锁存状态
 * @note  调用后传感器降频至 10Hz，开启硬件高通滤波器和锁存机制。
 *        STM32 主控在调用此函数并清空引脚残留后，即可安全进入深度睡眠。
 *
 * @param spi_spec 指向 SPI 配置规范
 * @param threshold_lsb 硬件触发阈值 (±2g量程下，1 LSB = 16mg，推荐 15U = 240mg)
 * @return int 成功返回 0，失败返回负数内核错误码
 */
int lis3dh_enter_low_power_wom(const struct spi_dt_spec *spi_spec, uint8_t threshold_lsb);

/**
 * @brief 🧠 核心新增：将 LIS3DH 快速切换回正常工作状态（高性能计步模式）
 * @note  当 STM32 被 GPIO 中断唤醒后，应立即调用此函数。
 *        传感器将被满血恢复至 25Hz/12-bit 高精度采样，并安全关闭硬件高通滤波器。
 *
 * @param spi_spec 指向 SPI 配置规范
 * @return int 成功返回 0，失败返回负数内核错误码
 */
int lis3dh_exit_to_normal_walking(const struct spi_dt_spec *spi_spec);

/**
 * @brief 读取并清除 INT1_SRC 寄存器以解锁中断锁存
 * @param[out] src 接收当前中断源状态的指针
 */
int lis3dh_clear_interrupt(const struct spi_dt_spec *spi_spec, uint8_t *src);

/**
 * @brief 刷新内置硬件高通滤波器基准线
 * @note  通过虚拟读取 REFERENCE 寄存器，将当前传感器所处的三轴空间重力瞬间归零作为全新静态基准
 */
int lis3dh_reset_baseline(const struct spi_dt_spec *spi_spec);

#endif /* LIS3DH_H */
