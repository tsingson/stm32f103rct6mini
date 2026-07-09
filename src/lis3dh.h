#ifndef LIS3DH_H
#define LIS3DH_H

#include <zephyr/drivers/spi.h>

/* LIS3DH 寄存器地址映射 */
#define LIS3DH_REG_WHO_AM_I      0x0F
#define LIS3DH_REG_CTRL_REG1     0x20
#define LIS3DH_REG_CTRL_REG3     0x22
#define LIS3DH_REG_CTRL_REG4     0x23
#define LIS3DH_REG_INT1_CFG      0x30
#define LIS3DH_REG_INT1_SRC      0x31
#define LIS3DH_REG_INT1_THS      0x32
#define LIS3DH_REG_INT1_DURATION 0x33
#define LIS3DH_REG_OUT_X_L       0x28

/**
 * @brief 初始化 LIS3DH 并配置为运动唤醒中断模式
 */
int lis3dh_init(const struct spi_dt_spec* spi_spec);

/**
 * @brief 通用寄存器读取
 */
int lis3dh_reg_read(const struct spi_dt_spec* spi_spec, uint8_t reg, uint8_t* data, size_t len);

/**
 * @brief 通用寄存器写入
 */
int lis3dh_reg_write(const struct spi_dt_spec* spi_spec, uint8_t reg, uint8_t data);

/**
 * @brief 读取 INT1_SRC 寄存器以清除中断锁存
 */
int lis3dh_clear_interrupt(const struct spi_dt_spec* spi_spec, uint8_t* src);

/* 刷新高通滤波器基准，将当前状态归零，并清空历史中断锁存 */
int lis3dh_reset_baseline(const struct spi_dt_spec* spi_spec);
//
int lis3dh_read_xyz(const struct spi_dt_spec* spi_spec, int16_t* x, int16_t* y, int16_t* z);
#endif /* LIS3DH_H */
