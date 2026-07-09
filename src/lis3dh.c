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

    /* 验证芯片身份 */
    ret = lis3dh_reg_read(spi_spec, LIS3DH_REG_WHO_AM_I, &who_am_i, 1);
    if (ret < 0 || (who_am_i != 0x33 && who_am_i != 0x3F))
    {
        return -ENODEV;
    }

    /* 1. CTRL_REG1: 100Hz 采样率, 开启 X/Y/Z 轴 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG1, 0x57);
    if (ret < 0) return ret;

    /* 2. 修正 CTRL_REG2: 写入 0x09
     * 开启 FDS(Bit3)=1 和 HP_IA1(Bit0)=1，全面激活中断引擎的高通滤波器 */
    ret = lis3dh_reg_write(spi_spec, 0x21, 0x09);
    if (ret < 0) return ret;

    /* 3. CTRL_REG3: 将 IA1 信号路由到物理 INT1 引脚 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG3, 0x40);
    if (ret < 0) return ret;

    /* 4. CTRL_REG5: 开启 LIR1 中断锁存，确保 STM32 不漏信号 */
    ret = lis3dh_reg_write(spi_spec, 0x24, 0x08);
    if (ret < 0) return ret;

    /* 5. INT1_THS: 设定阈值为 0x12 (约 288mg) */
    /**
     *日志里读取到的 Interrupt Source: 0x55 意味着芯片自身的 X/Y/Z 三个轴在高通滤波后依然同时超标。有些面包板的电源纹波或者芯片高通滤波器的初始噪声较大，0x12（288mg）可能还是有些低。我们把阈值提高到 0x20（约 512mg，大约半个重力加速度）
     */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_THS, 0x20);
    if (ret < 0) return ret;

    /* 6. INT1_DURATION: 0 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_DURATION, 0x00);
    if (ret < 0) return ret;

    /* 💡 核心新增：强制归零高通滤波器 */
    /* 通过读取 REFERENCE 寄存器(0x26)，强行让芯片把当前的静态重力当做 0 坐标 baseline */
    uint8_t dummy_ref;
    ret = lis3dh_reg_read(spi_spec, 0x26, &dummy_ref, 1);
    if (ret < 0) return ret;

    /* 7. INT1_CFG: 开启 X/Y/Z 高低阈值全监听 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_CFG, 0x3F);
    if (ret < 0) return ret;

    /* 预读清空标志 */
    uint8_t dummy;
    lis3dh_clear_interrupt(spi_spec, &dummy);

    return 0;
}

int lis3dh_clear_interrupt(const struct spi_dt_spec* spi_spec, uint8_t* src)
{
    return lis3dh_reg_read(spi_spec, LIS3DH_REG_INT1_SRC, src, 1);
}
