

# LIS3DH walk 步行测试

```
 
 #include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include "walk.h"

/* 直接通过设备树获取 I2C 总线设备 */
const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#define LIS3DH_I2C_ADDR  0x18  /* 芯片 I2C 地址 */
#define OUT_X_L          0x28  /* 数据寄存器起始地址，最高位置 1 支持自增读取 */
#define READ_CMD         (OUT_X_L | 0x80) 

int main(void) {
    uint8_t raw_data[6];
    int16_t x_raw, y_raw, z_raw;
    uint32_t total_steps = 0;

    walk_pedometer_t my_pedometer;
    /* 初始化：量程±2g，计步波动阈值 300 LSB，防抖 300ms */
    walk_pedometer_init(&my_pedometer, 2, 300, 300);

    if (!device_is_ready(i2c_dev)) {
        printf("I2C 设备未就绪\n");
        return -1;
    }

    // TODO: 外部在此处配置 LIS3DH 的寄存器（如 CTRL_REG1=0x37 启用 ODR 25Hz）

    while (1) {
        /* 快速低开销地直接读取 6 字节原始数据 */
        if (i2c_burst_read(i2c_dev, LIS3DH_I2C_ADDR, READ_CMD, raw_data, 6) == 0) {
            
            /* C17 标准下安全的原始字节拼接 */
            x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
            y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
            z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

            /* 调用纯整数快速算法 */
            int64_t now_ms = k_uptime_get();
            if (walk_pedometer_process(&my_pedometer, &x_raw, &y_raw, &z_raw, now_ms)) {
                total_steps++;
                printf("[FAST WALK] 步数 +1！当前总数: %u\n", total_steps);
            }
        }

        k_msleep(40); /* 对应 25Hz 采样率 */
    }
    return 0;
}

```
