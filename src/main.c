#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"

#define SAMPLING_RATE_MS    (40)
#define LIS3DH_NODE         DT_NODELABEL(lis3dsh)

static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DH_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
static const struct gpio_dt_spec int1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensor_irq_gpios);

int main(void)
{
    int16_t x_raw = 0, y_raw = 0, z_raw = 0;
    uint32_t total_steps = 0U, steps_inc = 0U;
    walk_pedometer_t my_pedometer;

    k_msleep(500);
    if (!spi_is_ready_dt(&spi_dev) || lis3dh_init(&spi_dev) < 0) return 0;
    lis3dh_reset_baseline(&spi_dev);
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | GPIO_PULL_DOWN);

    /* ================================================================= */
    /* 💡 全自动均值自适应初始化配置                                        */
    /* ================================================================= */
    uint32_t adaptive_debounce = 420U;   /* 420ms 基础迈步防抖去回弹 */
    uint32_t adaptive_timeout  = 4500U;  /* 放宽断步超时时间为 4.5 秒，纵容慢速走停 */
    uint32_t min_amplitude     = 140U;   /* 幅度阻尼：动作起伏低于 140 LSB 的纯杂噪直接过滤 */

    /* 一键配置，算法内部将自动跟踪 Peak-Valley 并划定中间动态门槛 */
    walk_pedometer_init(&my_pedometer, 2, adaptive_debounce, adaptive_timeout, min_amplitude);
    /* ================================================================= */

    while (1)
    {
        if (lis3dh_read_xyz(&spi_dev, &x_raw, &y_raw, &z_raw) == 0)
        {
            int64_t now_ms = k_uptime_get();
            (void)walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms, &steps_inc);

            if (steps_inc > 0U) {
                total_steps += steps_inc;
                if (steps_inc == (uint32_t)WALK_REQUIRED_STEPS) {
                    printf("\n🚀🚀🚀 [ADAPTIVE ACTIVE] 自动追踪步态成功！追加 %u 步。总步数: %u\n\n", WALK_REQUIRED_STEPS, total_steps);
                } else {
                    printf("🚶 [ADAPTIVE WALK] 规律波幅动态跨越，实时加步。总步数: %u\n", total_steps);
                }
            }
        }
        k_msleep(SAMPLING_RATE_MS);
    }
    return 0;
}
