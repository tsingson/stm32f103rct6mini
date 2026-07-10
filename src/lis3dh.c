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


int lis3dh_read_xyz(const struct spi_dt_spec* spi_spec, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t axis_data[6];
    /* 💡 核心：0x28 (OUT_X_L) | 0x80 (读指令) | 0x40 (多字节自增) = 0xC8 */
    uint8_t reg_addr = LIS3DH_REG_OUT_X_L | 0x80U | 0x40U;
    if (lis3dh_reg_read(spi_spec, reg_addr, axis_data, 6) == 0)
    {
        *x = ((int16_t)((axis_data[1] << 8) | axis_data[0]));
        *y = ((int16_t)((axis_data[3] << 8) | axis_data[2]));
        *z = ((int16_t)((axis_data[5] << 8) | axis_data[4]));
        // printk("   Accel Data -> X: %d | Y: %d | Z: %d\n", x, y, z);
        return 0;
    }
    return -1;
}


int lis3dh_enter_low_power_wom(const struct spi_dt_spec* spi_spec, uint8_t threshold_lsb)
{
    uint8_t dummy_src = 0U;

    /* 1. 先关闭中断使能，防止配置过程中的瞬时乱序电平误触发主控 */
    int ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_CFG, 0x00U);
    if (ret < 0) return ret;

    /* 2. 配置 CTRL_REG1: ODR = 10Hz, 正常模式(8-bit), 三轴使能 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG1, 0x17U);
    if (ret < 0) return ret;

    /* 3. 配置 CTRL_REG2: 开启硬件高通滤波器 (HPF) 引入中断 1 */
    ret = lis3dh_reg_write(spi_spec, 0x21U, 0x01U);
    if (ret < 0) return ret;

    /* 4. 配置 CTRL_REG3: 将 IA1 路由至 INT1 引脚 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG3, 0x40U);
    if (ret < 0) return ret;

    /* 5. 配置 CTRL_REG4: 量程 ±2g, 开启 LIR (中断锁存) */
    /* 锁存能保证电平持续为高，方便低功耗休眠中的 STM32 稳稳抓到中断沿 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG4, 0x08U);
    if (ret < 0) return ret;

    /* 6. 配置 INT1_THS: 设定硬件运动触发阈值 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_THS, threshold_lsb);
    if (ret < 0) return ret;

    /* 7. 配置 INT1_DURATION: 持续时间设为 1 个时钟周期 (10Hz下约100ms) */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_DURATION, 0x01U);
    if (ret < 0) return ret;

    /* 8. 核心必修步骤：刷新高通滤波器基准线，强行读一次清除锁存，把引脚压回绝对低电平 */
    (void)lis3dh_reset_baseline(spi_spec);
    (void)lis3dh_clear_interrupt(spi_spec, &dummy_src);

    /* 9. 【最后一步】：滤波器完全稳定后，再安全开启三轴高方向事件中断使能，规避自激幽灵中断 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_CFG, 0x2AU);

    return ret;
}

int lis3dh_exit_to_normal_walking(const struct spi_dt_spec* spi_spec)
{
    int ret;

    /* 1. 立即关闭 INT1 的硬件中断使能，防止后续快速计步时，引脚频繁跳变干扰 STM32 的中断服务函数 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_INT1_CFG, 0x00U);
    if (ret < 0) return ret;

    /* 2. 满血复活配置 CTRL_REG1: ODR = 25Hz, 12-bit 高分辨率模式 */
    ret = lis3dh_reg_write(spi_spec, LIS3DH_REG_CTRL_REG1, 0x37U);
    if (ret < 0) return ret;

    /* 3. 关闭硬件内置高通滤波器（交由 walk.c 的软件自适应均值窗口进行更高级的滤波） */
    ret = lis3dh_reg_write(spi_spec, 0x21U, 0x00U);

    return ret;
}

/**
 * @brief 刷新内置硬件高通滤波器基准线
 * @note  通过虚拟读取 REFERENCE (0x26) 寄存器，强行令硬件瞬间对齐当前空间重力
 */
int lis3dh_reset_baseline(const struct spi_dt_spec* spi_spec)
{
    uint8_t dummy_data = 0U;
    int ret;

    /* 根据 LIS3DH SPI 协议：读取单寄存器时，最高位 (Bit 0) 必须置 1 (0x80) */
    /* 0x26 | 0x80 = 0xA6 */
    uint8_t reg_addr = LIS3DH_REG_REFERENCE | 0x80U;

    /* 构建 Zephyr 标准 SPI 读写缓冲区 */
    struct spi_buf tx_buf = {.buf = &reg_addr, .len = 1U};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1U};

    /* dummy_data 用于接收读取出来的无用基准字节 */
    struct spi_buf rx_buf = {.buf = &dummy_data, .len = 1U};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1U};

    /* 执行 SPI 事务：先发寄存器地址，再读取内容 */
    ret = spi_write_dt(spi_spec, &tx_set);
    if (ret < 0)
    {
        return ret;
    }

    ret = spi_read_dt(spi_spec, &rx_set);
    if (ret < 0)
    {
        return ret;
    }

    /* 显式强转抛弃 dummy_data 的未使用警告，符合 C17 严苛的静态分析规范 */
    (void)dummy_data;

    return 0; /* 成功返回 0，硬件 HPF 零位线已成功刷新 */
}
