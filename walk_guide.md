

# LIS3DH walk 步行测试

```
 #include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"

#define SAMPLING_RATE_MS    (40)
static const struct spi_dt_spec lis3dh_spi = SPI_DT_SPEC_GET(DT_NODELABEL(lis3dh), SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);

int main(void) {
    int16_t x_raw = 0;
    int16_t y_raw = 0;
    int16_t z_raw = 0;
    uint32_t total_steps = 0U;
    uint32_t steps_inc = 0U;

    if (!spi_is_ready_dt(&lis3dh_spi)) {
        return -EINVAL;
    }
    if (lis3dh_init(&lis3dh_spi) < 0) {
        return -EIO;
    }

    walk_pedometer_t my_pedometer;
    walk_pedometer_init(&my_pedometer, 2, 300, 300);
    lis3dh_reset_baseline(&lis3dh_spi);

    while (1) {
        /* 完全信任并在内部传递指针，不进行外部重复拼接 */
        if (lis3dh_read_xyz(&lis3dh_spi, &x_raw, &y_raw, &z_raw) == 0) {
            int64_t now_ms = k_uptime_get();

            /* 参数值传递：安全传递 x_raw, y_raw, z_raw 副本 */
            (void)walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms, &steps_inc);

            if (steps_inc > 0U) {
                total_steps += steps_inc;
                if (steps_inc == WALK_REQUIRED_STEPS) {
                    printf("[WALK TRIGGER] 连续走满4步激活！追加4步。当前总数: %u\n", total_steps);
                } else {
                    printf("[WALK] 实时计步。当前总数: %u\n", total_steps);
                }
            }
        }
        k_msleep(SAMPLING_RATE_MS);
    }
    return 0;
}

```
