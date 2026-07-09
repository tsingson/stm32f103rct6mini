#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define LIS3DSH_NODE DT_NODELABEL(lis3dsh)

/*
 * 默认使用 Mode 3 (CPOL=1, CPHA=1)。
 * 如果依然读出 0x00，可以尝试把后面的 | SPI_MODE_CPOL | SPI_MODE_CPHA 去掉切换回 Mode 0。
 */
static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DSH_NODE,
                                          SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
                                          0);

#define LIS3DSH_REG_WHO_AM_I   0x0F
#define LIS3DSH_REG_CTRL4      0x20
#define LIS3DSH_REG_OUT_X_L    0x28

#define SPI_READ_BIT           0x80
#define SPI_AUTO_INC_BIT       0x40

/* 单字节写入 */
static int lis3dsh_reg_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = { reg, data };
    const struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    return spi_write_dt(&spi_dev, &tx_set);
}

/* 多字节读取 (关键修正：TX/RX 长度完全对齐) */
static int lis3dsh_reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[8] = {0};

    if (len > 7) {
        return -EINVAL;
    }

    /* 构造发送指令 */
    tx_buf[0] = reg | SPI_READ_BIT;
    if (len > 1) {
        tx_buf[0] |= SPI_AUTO_INC_BIT;
    }

    /* 核心修正：len + 1 使得双向缓冲区长度完全一致 */
    const struct spi_buf tx = { .buf = tx_buf, .len = len + 1 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    const struct spi_buf rx = { .buf = rx_buf, .len = len + 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int ret = spi_transceive_dt(&spi_dev, &tx_set, &rx_set);
    if (ret == 0) {
        /* 剥离掉第 0 字节的 dummy 数据，把真实传感器数据拷回 */
        memcpy(data, &rx_buf[1], len);
    }
    return ret;
}

int main(void)
{
    int ret;
    uint8_t who_am_i = 0;
    uint8_t axis_data[6] = {0}; // 存放 X_L, X_H, Y_L, Y_H, Z_L, Z_H

    printk("Starting LIS3DSH SPI Test...\n");

    if (!spi_is_ready_dt(&spi_dev)) {
        printk("Error: SPI device is not ready!\n");
        return 0;
    }

    /*
     * 【硬件排查利器】死循环读取 WHO_AM_I
     * LIS3DSH 预期是 0x3F，如果是 LIS3DH 预期是 0x33
     */
    while (1) {
        ret = lis3dsh_reg_read(LIS3DSH_REG_WHO_AM_I, &who_am_i, 1);
        if (ret == 0 && (who_am_i == 0x3F || who_am_i == 0x33)) {
            printk("Successfully detected sensor! WHO_AM_I = 0x%02X\n", who_am_i);
            break;
        }
        printk("Searching for sensor... Read WHO_AM_I = 0x%02X. Check wiring!\n", who_am_i);
        k_msleep(1000);
    }

    /* 初始化传感器 (CTRL_REG4: 100Hz, 开启 X/Y/Z) */
    ret = lis3dsh_reg_write(LIS3DSH_REG_CTRL4, 0x67);
    if (ret < 0) {
        printk("Failed to initialize LIS3DSH\n");
        return 0;
    }

    while (1) {
        ret = lis3dsh_reg_read(LIS3DSH_REG_OUT_X_L, axis_data, 6);

        int16_t x_raw = (int16_t)((axis_data[1] << 8) | axis_data[0]);
        int16_t y_raw = (int16_t)((axis_data[3] << 8) | axis_data[2]);
        int16_t z_raw = (int16_t)((axis_data[5] << 8) | axis_data[4]);

        /* 如果发现三轴数据全是 0 */
        if (x_raw == 0 && y_raw == 0 && z_raw == 0) {
            uint8_t check_id = 0;
            lis3dsh_reg_read(LIS3DSH_REG_WHO_AM_I, &check_id, 1);
            printk("[Warning] Data is ZERO! Checking WHO_AM_I = 0x%02X\n", check_id);

            /* 尝试重新向配置寄存器写 0x67 把它唤醒 */
            lis3dsh_reg_write(LIS3DSH_REG_CTRL4, 0x67);
        } else {
            /* 正常打印数据 */
            float x_g = (float)x_raw * 0.061f / 1000.0f;
            float y_g = (float)y_raw * 0.061f / 1000.0f;
            float z_g = (float)z_raw * 0.061f / 1000.0f;
            printk("X: %7.5f g | Y: %7.5f g | Z: %7.5f g\n", x_g, y_g, z_g);
        }

        k_msleep(500);
    }
}
