#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"

#define SAMPLING_RATE_MS    (40)
#define LIS3DH_NODE         DT_NODELABEL(lis3dsh)

static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DH_NODE,
                                                          SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
                                                          0);

static const struct gpio_dt_spec int1_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensor_irq_gpios);

K_SEM_DEFINE(motion_sem, 0, 1);
static struct gpio_callback int1_cb_data;

void int1_gpio_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    /* 显式消除未定义变量警告 */
    (void)dev;
    (void)cb;
    (void)pins;
    k_sem_give(&motion_sem);
}

int main(void)
{
    int16_t x_raw = 0;
    int16_t y_raw = 0;
    int16_t z_raw = 0;
    uint32_t total_steps = 0U;
    uint32_t steps_inc = 0U;
    int ret;
    uint8_t int_src = 0;
    uint8_t who_am_i = 0;
    walk_pedometer_t my_pedometer;

    /* 集中收拢老人调优变量 */
    uint32_t elderly_threshold = 180U;
    uint32_t elderly_debounce  = 400U;
    uint32_t elderly_timeout   = 4000U;
    uint32_t log_counter = 0U;

    k_msleep(500);
    printk("\n--- LIS3DH Perfect Wake-On-Motion Test ---\n");

    if (!spi_is_ready_dt(&spi_dev) || !gpio_is_ready_dt(&int1_gpio))
    {
        printk("Error: Hardware not ready!\n");
        return 0;
    }

    ret = lis3dh_init(&spi_dev);
    if (ret < 0)
    {
        printk("Sensor init failed: %d\n", ret);
        return 0;
    }

    k_msleep(200);
    ret = lis3dh_reg_read(&spi_dev, 0x0F, &who_am_i, 1);
    if (ret < 0)
    {
        printk("LIS3DSH WHO_AM_I read failed: %d\n", ret);
        return ret;
    }
    else
    {
        printk("LIS3DSH WHO_AM_I: 0x%02X\n", who_am_i);
    }

    ret = lis3dh_reset_baseline(&spi_dev);
    if (ret < 0)
    {
        printk("Reset baseline failed: %d\n", ret);
        return 0;
    }

    k_msleep(500);
    lis3dh_clear_interrupt(&spi_dev, &int_src);
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | GPIO_PULL_DOWN);

    printk("System Silent. STM32 is sleeping... Tap/Move the board now!\n");
    printf("LIS3DH 硬件初始化成功，进入计步核心监测循环...\n");

    walk_pedometer_init(&my_pedometer, 2, elderly_threshold, elderly_debounce, elderly_timeout);
    lis3dh_reset_baseline(&spi_dev);

    while (1)
    {
        if (lis3dh_read_xyz(&spi_dev, &x_raw, &y_raw, &z_raw) == 0)
        {
            int64_t now_ms = k_uptime_get();
            bool is_walking = walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms, &steps_inc);

            log_counter++;
            if (log_counter % 5U == 0U) {
                /* 修正：将计算开方和滤波均值的代码块变量全部规范隔离，严防 C17 混合作用域警告 */
                uint32_t ux = (x_raw < 0) ? (uint32_t)(-x_raw) : (uint32_t)x_raw;
                uint32_t uy = (y_raw < 0) ? (uint32_t)(-y_raw) : (uint32_t)y_raw;
                uint32_t uz = (z_raw < 0) ? (uint32_t)(-z_raw) : (uint32_t)z_raw;
                uint32_t v_val = (ux * ux) + (uy * uy) + (uz * uz);
                uint32_t bit = 1U << 30;
                uint32_t res = 0U;
                uint32_t high_bound = my_pedometer.gravity_base + my_pedometer.step_threshold;

                while (bit > v_val) {
                    bit >>= 2;
                }
                while (bit != 0U) {
                    if (v_val >= res + bit) {
                        v_val -= res + bit;
                        res = (res >> 1) + bit;
                    } else {
                        res >>= 1;
                    }
                    bit >>= 2;
                }

                printf("[DEBUG_WAVE] 原始值:%u, 滤波值:%d, 触发门槛:%u, 缓冲池步数:%u, 行走激活状态:%d\n",
                       res,
                       (int)(my_pedometer.filter_sum / (int32_t)FILTER_WINDOW_SIZE),
                       high_bound,
                       my_pedometer.continuous_steps,
                       is_walking);
            }

            if (steps_inc > 0U)
            {
                total_steps += steps_inc;
                /* 修正：通过显式接口比对或统一强强类型比对，防止解耦破缺 */
                if (steps_inc == (uint32_t)WALK_REQUIRED_STEPS)
                {
                    printf("\n🚀🚀🚀 [WALK TRIGGER SUCCESS] 连续走满 %u 步激活！追加追偿步数。当前总步数: %u\n\n",
                           WALK_REQUIRED_STEPS, total_steps);
                }
                else
                {
                    printf("🚶 [WALK CONTINUOUS] 步态持续中，实时计步 +1。当前总步数: %u\n", total_steps);
                }
            }
        }
        k_msleep(SAMPLING_RATE_MS);
    }
    return 0;
}
