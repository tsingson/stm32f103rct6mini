#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"

#define SAMPLING_RATE_MS    (40)
static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(DT_NODELABEL(lis3dsh), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
static const struct gpio_dt_spec int1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensor_irq_gpios);

K_SEM_DEFINE(motion_sem, 0, 1);
static struct gpio_callback int1_cb_data;

void int1_gpio_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    (void)dev;
    (void)cb;
    (void)pins;
    k_sem_give(&motion_sem);
}

int main(void)
{
    int16_t x_raw = 0, y_raw = 0, z_raw = 0;
    uint32_t total_steps = 0U, steps_inc = 0U;
    uint8_t int_src = 0;
    walk_pedometer_t my_pedometer;

    /* 初始化全自动自适应老人计步器 */
    walk_pedometer_init(&my_pedometer, 2, 420U, 4500U, 140U);

    if (!spi_is_ready_dt(&spi_dev) || !gpio_is_ready_dt(&int1_gpio))
    {
        return 0;
    }

    /* 配置 STM32 GPIO 中断监听 */
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
    gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&int1_cb_data, int1_gpio_isr, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);

    while (1)
    {
        /* ================================================================= */
        /* STATUS A: 超低功耗 WOM 休眠准备阶段                                 */
        /* ================================================================= */
        printf("\n💤 [POWER] 配置 LIS3DH WOM 中断锁存... \n");
        k_msleep(5); // 规避大块串口数据阻塞引发的瞬间微秒时钟抖动

        if (lis3dh_enter_low_power_wom(&spi_dev, 15U) < 0)
        {
            k_msleep(500);
            continue;
        }

        /* 🛡️ 核心修复：在上机挂起前，强行将重置信号量消耗掉（清空残留），并最后刷一次传感器锁存 */
        (void)k_sem_reset(&motion_sem);
        (void)lis3dh_clear_interrupt(&spi_dev, &int_src);

        printf("💤 [POWER] 引脚底电平净化完毕。STM32 进入挂起睡眠...\n");
        k_msleep(5);

        /* 🔒 核心睡眠锚点：主控放弃 CPU，进入极致 µA 级深睡，静静等待老人迈出第一步 */
        (void)k_sem_take(&motion_sem, K_FOREVER);

        /* ================================================================= */
        /* STATUS B: 满血复活，恢复 25Hz 高性能计步状态                          */
        /* ================================================================= */
        printf("⏰ [POWER] 检测到人体运动中断！主控唤醒，提升全功能采样频率...\n");

        if (lis3dh_exit_to_normal_walking(&spi_dev) < 0)
        {
            continue;
        }

        /* 唤醒后立刻清除一次锁存高电平，确保引脚在整个计步期间回到就绪态 */
        (void)lis3dh_clear_interrupt(&spi_dev, &int_src);

        /* 设置 6 秒持续不活动退出时间窗 */
        int64_t last_active_time = k_uptime_get();

        while (1)
        {
            int64_t now_ms = k_uptime_get();

            /* 修正：对时间戳生存期差值执行严格的防御性范围计算，规避 C17 有符号优化截断 */
            int64_t time_elapsed = now_ms - last_active_time;
            uint64_t idle_duration_ms = (time_elapsed < 0) ? 0ULL : (uint64_t)time_elapsed;

            if (idle_duration_ms > 6000ULL)
            {
                /* 持续 6 秒没有有效活动步态，打破当前满血循环，准备退回 Status A */
                break;
            }

            if (lis3dh_read_xyz(&spi_dev, &x_raw, &y_raw, &z_raw) == 0)
            {
                bool is_walking = walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms, &steps_inc);

                if (steps_inc > 0U)
                {
                    total_steps += steps_inc;
                    last_active_time = now_ms; /* 步数真实增加，刷新活跃地平线 */
                    printf("🚶 [WALK] 实时累计步数: %u\n", total_steps);
                }

                if (is_walking)
                {
                    last_active_time = now_ms; /* 处于迈步防误判缓冲期，同样视为人正在活动，刷新地平线 */
                }
            }
            k_msleep(SAMPLING_RATE_MS);
        }

        printf("⏳ [POWER] 连续 6 秒未发生步伐位移。正在熔断计步器，准备重新深睡...\n");
    }
    return 0;
}
