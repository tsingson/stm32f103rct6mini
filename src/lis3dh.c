#include "lis3dh.h"
#include <string.h>

#define SPI_READ_BIT           0x80
#define SPI_AUTO_INC_BIT       0x40

int lis3dh_reg_write(const struct spi_dt_spec* spi_spec, uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg, data};
    const struct spi_buf tx = {.buf = tx_buf, .len = 2};
    const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    return spi_write_dt(spi_spec, &tx_set);
}

int lis3dh_reg_read(const struct spi_dt_spec* spi_spec, uint8_t reg, uint8_t* data, size_t len)
{
    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[8] = {0};

    if (len > 7)
    {
        return -EINVAL;
    }

    tx_buf[0] = reg | SPI_READ_BIT;
    if (len > 1)
    {
        tx_buf[0] |= SPI_AUTO_INC_BIT;
    }

    const struct spi_buf tx = {.buf = tx_buf, .len = len + 1};
    const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    const struct spi_buf rx = {.buf = rx_buf, .len = len + 1};
    const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    int ret = spi_transceive_dt(spi_spec, &tx_set, &rx_set);
    if (ret == 0)
    {
        memcpy(data, &rx_buf[1], len);
    }
    return ret;
}

int lis3dh_init(const struct spi_dt_spec* spi_spec)
{
    uint8_t who_am_i = 0;
    int ret;

    /* 1. 验证 ID */
    ret = lis3dh_reg_read(spi_spec, 0x0F, &who_am_i, 1);
    if (ret < 0 || who_am_i != 0x33)
    {
        return -ENODEV;
    }

    /* 2. 暴力重置：写入 0x40 到 CTRL_REG5 (0x24)，触发软复位，清空所有残留配置 */
    lis3dh_reg_write(spi_spec, 0x24, 0x80);
    k_msleep(50); // 给芯片一点时间完成重启

    /* 3. CTRL_REG1: 100Hz, 开启 X/Y/Z 轴 */
    lis3dh_reg_write(spi_spec, 0x20, 0x57);

    /* 4. CTRL_REG2: 开启高通滤波路由到中断1，过滤重力 */
    lis3dh_reg_write(spi_spec, 0x21, 0x09);

    /* 5. CTRL_REG3: IA1 路由到物理 INT1 */
    lis3dh_reg_write(spi_spec, 0x22, 0x40);

    /* 6. CTRL_REG5: 开启锁存(LIR1) */
    lis3dh_reg_write(spi_spec, 0x24, 0x08);

    /* 7. 关键：INT1_THS 设置阈值 */
    lis3dh_reg_write(spi_spec, 0x32, 0x20);

    /* 8. 关键：INT1_CFG 只开启 High 事件 (0x2A) */
    lis3dh_reg_write(spi_spec, 0x30, 0x2A);

    /* 9. 强制清空一次中断状态，确保开机时 INT1 引脚是 0V */
    uint8_t dummy;
    lis3dh_clear_interrupt(spi_spec, &dummy);

    return 0;
}

int lis3dh_clear_interrupt(const struct spi_dt_spec* spi_spec, uint8_t* src)
{
    return lis3dh_reg_read(spi_spec, LIS3DH_REG_INT1_SRC, src, 1);
}

int lis3dh_reset_baseline(const struct spi_dt_spec* spi_spec)
{
    uint8_t dummy;
    int ret;

    /* 1. 在芯片稳定后读取 REFERENCE 寄存器，强行把当前的静态重力作为 0 坐标 baseline */
    ret = lis3dh_reg_read(spi_spec, 0x26, &dummy, 1);
    if (ret < 0) return ret;

    /* 2. 顺便通过读取 INT1_SRC 清空一次当前可能已经卡死的中断引脚 */
    return lis3dh_clear_interrupt(spi_spec, &dummy);
}

int lis3dh_read_xyz(const struct spi_dt_spec* spi_spec, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t axis_data[6];
    if (lis3dh_reg_read(spi_spec, LIS3DH_REG_OUT_X_L, axis_data, 6) == 0)
    {
        *x = ((int16_t)((axis_data[1] << 8) | axis_data[0]));
        *y = ((int16_t)((axis_data[3] << 8) | axis_data[2]));
        *z = ((int16_t)((axis_data[5] << 8) | axis_data[4]));
        // printk("   Accel Data -> X: %d | Y: %d | Z: %d\n", x, y, z);
        return 0;
    }
    return -1;
}
