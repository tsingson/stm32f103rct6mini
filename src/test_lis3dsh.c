#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h> /* 显式包含 GPIO 头文件，防止底层符号未定义 */
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"

#define SAMPLING_RATE_MS    (40)
/* 💡 修正隐患 1：标签去掉多余的 's'，完美匹配设备树标准定义的 'lis3dh' */
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
    (void)dev; (void)cb; (void)pins;
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

    uint32_t elderly_threshold = 180U;
    uint32_t elderly_debounce  = 400U;
    uint32_t elderly_timeout   = 4000U;
    uint32_t log_counter = 0U;

    k_msleep(500);
    printk("\n--- LIS3DH Hardened Low-Power Pedometer System ---\n");

    /* 验证底层硬件总线就绪 */
    if (!spi_is_ready_dt(&spi_dev) || !gpio_is_ready_dt(&int1_gpio))
    {
        printk("Error: Hardware peripherals not ready!\n");
        return 0;
    }

    ret = lis3dh_init(&spi_dev);
    if (ret < 0)
    {
        printk("Sensor hardware init failed: %d\n", ret);
        return 0;
    }

    k_msleep(200);
    ret = lis3dh_reg_read(&spi_dev, LIS3DH_REG_WHO_AM_I, &who_am_i, 1);
    if (ret < 0)
    {
        printk("LIS3DH WHO_AM_I read failed: %d\n", ret);
        return ret;
    }
    else
    {
        printk("LIS3DH WHO_AM_I Successfully Detected: 0x%02X\n", who_am_i);
    }

    /* 💡 修正隐患 3：此阶段仅配置输入方向与下拉，坚决不开启中断，严防开机模拟噪点虚假唤醒 */
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
    gpio_init_callback(&int1_cb_data, int1_gpio_isr, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);

    /* 初始化自适应核心上下文 */
    walk_pedometer_init(&my_pedometer, 2, elderly_debounce, elderly_timeout, elderly_threshold);

    while (1)
    {
        /* ================================================================= */
        /* STATUS A: 超低功耗 WOM 休眠配置阶段                                 */
        /* ================================================================= */
        printf("\n💤 [POWER_STATE] 正在配置传感器转入 WOM 10Hz 挂起模式...\n");
        k_msleep(5);

        /* 写入低功耗 WOM 配置寄存器流 */
        if (lis3dh_enter_low_power_wom(&spi_dev, 15U) < 0) {
            k_msleep(500);
            continue;
        }

        /* 🛡️ 极致衔接安全：在主控彻底睡下前，清空信号量残留，并清除一次硬件锁存高电平 */
        (void)k_sem_reset(&motion_sem);
        (void)lis3dh_clear_interrupt(&spi_dev, &int_src);

        /* 💡 核心修正 3：在引脚电平被完全净化为纯净低电平的这一瞬间，再开启 STM32 的中断监听 */
        gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

        printf("💤 [POWER_STATE] 物理总线已完全净化。STM32 进入挂起休眠状态...\n");
        k_msleep(5);

        /* 🔒 线程深度锚定：主控挂起，放弃 CPU，等待老人迈出第一步触发硬件中断 */
        (void)k_sem_take(&motion_sem, K_FOREVER);

        /* ================================================================= */
        /* STATUS B: 被动唤醒，切换为 25Hz 高性能自适应计步状态                   */
        /* ================================================================= */
        printf("⏰ [POWER_STATE] 捕捉到运动中断！主控满血复活，恢复 25Hz 采样...\n");

        /* 💡 核心修正 3：醒来后第一时间关闭 STM32 的 GPIO 中断监听，防止后续快速走路摆手时频繁进出 ISR 拖慢计步 */
        gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_DISABLE);

        if (lis3dh_exit_to_normal_walking(&spi_dev) < 0) {
            continue;
        }

        /* 清除唤醒时产生的锁存电平，复位总线 */
        (void)lis3dh_clear_interrupt(&spi_dev, &int_src);

        /* 设定 6 秒持续静止不活动超时机制 */
        int64_t last_active_time = k_uptime_get();

        while (1)
        {
            int64_t now_ms = k_uptime_get();

            /* 💡 修正隐患 4：执行无符号转换运算，确保时间差绝对处于防御性安全标量域 */
            int64_t time_elapsed = now_ms - last_active_time;
            uint64_t idle_duration_ms = (time_elapsed < 0) ? 0ULL : (uint64_t)time_elapsed;

            if (idle_duration_ms > 6000ULL) {
                /* 连续 6 秒未发生有效步伐位移或前置运动，打破循环，重新退回 Status A 深睡 */
                break;
            }

            if (lis3dh_read_xyz(&spi_dev, &x_raw, &y_raw, &z_raw) == 0)
            {
                /* 💡 修正隐患 4：使用标准承接标量，严防 (void) 强转副作用警告 */
                bool is_walking = walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms, &steps_inc);

                if (steps_inc > 0U) {
                    total_steps += steps_inc;
                    last_active_time = now_ms; /* 步数真实变动，刷新活跃时间戳 */
                    printf("🚶 [WALK_ENGINE] 精准捕捉成功。当前总步数: %u\n", total_steps);
                }

                if (is_walking) {
                    last_active_time = now_ms; /* 处于迈步防误判缓冲期，判定人仍处于活动状态，刷新时间戳 */
                }
            }
            k_msleep(SAMPLING_RATE_MS);
        }

        printf("⏳ [POWER_STATE] 持续静止超时。开始重置滤波器，准备进入下一轮深睡...\n");
    }
    return 0;
}
