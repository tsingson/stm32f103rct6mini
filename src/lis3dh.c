#include "lis3dh.h"
#include <string.h>

#define SPI_READ_BIT           0x80
#define SPI_AUTO_INC_BIT       0x40

int lis3dh_reg_write(const struct spi_dt_spec *spi_spec, uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = { reg, data };
    const struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    return spi_write_dt(spi_spec, &tx_set);
}

/* 核心修复：这里的参数类型、指针 *、变量名 spi_spec 已经完全修正，与头文件绝对对齐 */
int lis3dh_reg_read(const struct spi_dt_spec *spi_spec, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[8] = {0};

    if (len > 7) {
        return -EINVAL;
    }

    tx_buf[0] = reg | SPI_READ_BIT;
    if (len > 1) {
        tx_buf[0] |= SPI_AUTO_INC_BIT;
    }

    const struct spi_buf tx = { .buf = tx_buf, .len = len + 1 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    const struct spi_buf rx = { .buf = rx_buf, .len = len + 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive_dt(spi_spec, &tx_set, &rx_set);
    if (ret == 0) {
        memcpy(data, &rx_buf[1], len);
    }
    return ret;
}

int lis3dh_init(const struct spi_dt_spec *spi_spec)
{
    uint8_t who_am_i = 0;
    int ret;

    /* 验证芯片身份 */
    ret = lis3dh_reg_read(spi_spec, LIS3DH_REG_WHO_AM_I, &who_am_i, 1);
    if (ret < 0 || (who_am_i != 0x33 && who_am_i != 0x3F)) {
        return -ENODEV;
    }

    /* 1. CTRL_REG1: 100Hz 采样率, 开启 X/Y/Z 轴 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG1, 0x57);
    if (ret < 0) return ret;

    /* 2. CTRL_REG3: 将 IA1（中断1活动）信号路由到物理 INT1 引脚 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG3, 0x40);
    if (ret < 0) return ret;

    /* 3. INT1_THS: 中断阈值 (±2g量程下 1 LSB = 16mg)
       0x10 对应 16 * 16mg = 256mg (约0.25g)，轻微晃动即可触发 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_THS, 0x10);
    if (ret < 0) return ret;

    /* 4. INT1_DURATION: 持续时间设为 0，只要有 1 个采样点超标立刻中断 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_DURATION, 0x00);
    if (ret < 0) return ret;

    /* 5. INT1_CFG: 开启 X高、Y高、Z高 阈值事件的“或(OR)”组合（Any-Motion） */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_CFG, 0x2A);
    if (ret < 0) return ret;

    /* 预读一次，清除上电时可能残留的中断状态 */
    uint8_t dummy;
    lis3dh_clear_interrupt(spi_spec, &dummy);

    return 0;
}

int lis3dh_clear_interrupt(const struct spi_dt_spec *spi_spec, uint8_t *src)
{
    /* LIS3DH 硬件规定：读取 INT1_SRC 寄存器会自动将物理 INT1 引脚恢复为低电平 */
    return lis3dh_reg_read(spi_spec, LIS3DH_REG_INT1_SRC, src, 1);
}
