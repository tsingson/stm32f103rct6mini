#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

/*
 * 从设备树中获取 lis3dsh 节点。
 * 对应你 overlay 中的 lis3dsh: lis3dsh@0
 */
#define LIS3DSH_NODE DT_NODELABEL(lis3dsh)

/*
 * 定义 SPI 设备规格 (包含 CS 引脚控制)。
 * LIS3DSH 支持 CPOL=1, CPHA=1 (Mode 3) 或 CPOL=0, CPHA=0 (Mode 0)。
 */
static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DSH_NODE,
                                          SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
                                          0);

/* LIS3DSH 寄存器地址 */
#define LIS3DSH_REG_WHO_AM_I   0x0F
#define LIS3DSH_REG_CTRL4      0x20
#define LIS3DSH_REG_OUT_X_L    0x28

/* ST 传感器的 SPI 读写位和地址自增位 */
#define SPI_READ_BIT           0x80
#define SPI_AUTO_INC_BIT       0x40

/**
 * @brief 向 LIS3DSH 写入单个寄存器
 */
static int lis3dsh_reg_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = { reg, data }; // 最高位为 0 表示写入
    const struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    return spi_write_dt(&spi_dev, &tx_set);
}

/**
 * @brief 从 LIS3DSH 读取多个寄存器
 */
static int lis3dsh_reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    /*
     * 若读取多个字节，需要开启地址自增位 (SPI_AUTO_INC_BIT)
     * 并设置读取位 (SPI_READ_BIT)
     */
    uint8_t tx_cmd = reg | SPI_READ_BIT;
    if (len > 1) {
        tx_cmd |= SPI_AUTO_INC_BIT;
    }

    uint8_t tx_buf[1] = { tx_cmd };
    const struct spi_buf tx = { .buf = tx_buf, .len = 1 };
    const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    /* 接收缓冲区需要比实际数据多 1 个字节（为了应对发送命令时的虚拟接收） */
    const struct spi_buf rx = { .buf = data, .len = len + 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    return spi_transceive_dt(&spi_dev, &tx_set, &rx_set);
}

int main(void)
{
    int ret;
    uint8_t rx_buf[7] = {0}; // 1 字节 dummy + 6 字节数据 (X, Y, Z 各两个字节)

    printk("Starting LIS3DSH SPI Test...\n");

    /* 1. 检查 SPI 设备是否就绪 */
    if (!spi_is_ready_dt(&spi_dev)) {
        printk("Error: SPI device is not ready!\n");
        return 0;
    }

    /* 2. 读取 WHO_AM_I 寄存器确认芯片 (LIS3DSH 的预期值是 0x3F) */
    ret = lis3dsh_reg_read(LIS3DSH_REG_WHO_AM_I, rx_buf, 1);
    if (ret < 0) {
        printk("Failed to read WHO_AM_I (err %d)\n", ret);
        return 0;
    }
    printk("LIS3DSH WHO_AM_I: 0x%02X (Expected: 0x3F)\n", rx_buf[1]);

    /* 3. 初始化传感器 (CTRL_REG4: 100Hz 输出，开启 X, Y, Z) */
    // 0x67 = 0110 (100Hz) 0111 (Z, Y, X enable)
    ret = lis3dsh_reg_write(LIS3DSH_REG_CTRL4, 0x67);
    if (ret < 0) {
        printk("Failed to initialize LIS3DSH\n");
        return 0;
    }
    printk("LIS3DSH Initialized.\n");
    k_msleep(100); // 等待传感器稳定

    /* 4. 循环读取三轴数据 */
    while (1) {
        ret = lis3dsh_reg_read(LIS3DSH_REG_OUT_X_L, rx_buf, 6);
        if (ret == 0) {
            /*
             * 数据组合：低位在前，高位在后。
             * rx_buf[0] 是发送指令时的 dummy 数据，真实数据从 rx_buf[1] 开始。
             */
            int16_t x_raw = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
            int16_t y_raw = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
            int16_t z_raw = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);

            /* 转换为以 g 为单位的物理值 (假设默认 ±2g 量程，灵敏度约为 0.061 mg/LSB) */
            float x_g = (float)x_raw * 0.061f / 1000.0f;
            float y_g = (float)y_raw * 0.061f / 1000.0f;
            float z_g = (float)z_raw * 0.061f / 1000.0f;

            printk("X: %7.3f g | Y: %7.3f g | Z: %7.3f g\n", x_g, y_g, z_g);
        } else {
            printk("SPI Read Error: %d\n", ret);
        }

        k_msleep(500); // 每 500ms 打印一次
    }
}
