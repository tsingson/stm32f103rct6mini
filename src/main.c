#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"

/* 采样率匹配：行走频率通常低于 3Hz，25Hz 采样率 (40ms 间隔) 是兼顾功耗与精度的黄金选项 */
#define SAMPLING_RATE_MS    (40)
#define LIS3DH_NODE DT_NODELABEL(lis3dsh)

static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DH_NODE,
                                                          SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
                                                          0);

/* 方案 2 的 zephyr,user 路径获取引脚 */
static const struct gpio_dt_spec int1_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensor_irq_gpios);

K_SEM_DEFINE(motion_sem, 0, 1);
static struct gpio_callback int1_cb_data;

void int1_gpio_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    k_sem_give(&motion_sem);
}


int main(void)
{
    k_msleep(500);
    //
    int16_t x_raw = 0;
    int16_t y_raw = 0;
    int16_t z_raw = 0;
    uint32_t total_steps = 0;
    //

    int ret;
    uint8_t int_src = 0;

    printk("\n--- LIS3DH Perfect Wake-On-Motion Test ---\n");

    if (!spi_is_ready_dt(&spi_dev) || !gpio_is_ready_dt(&int1_gpio))
    {
        printk("Error: Hardware not ready!\n");
        return 0;
    }

    // 1. 初始化 LIS3DH 寄存器配置（此时传感器开始通电，滤波器开始工作）
    ret = lis3dh_init(&spi_dev);
    if (ret < 0)
    {
        printk("Sensor init failed: %d\n", ret);
        return 0;
    }

    // 2. 核心关键：让子弹飞一会儿！静止等待 200ms，让高通滤波器彻底“吃掉”并稳定地球重力
    k_msleep(200);

    uint8_t who_am_i = 0;
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

    // 3. 稳准狠：此时传感器已完全稳定，执行校准，扣除重力基准并强行释放 INT1 高电平
    ret = lis3dh_reset_baseline(&spi_dev);
    if (ret < 0)
    {
        printk("Reset baseline failed: %d\n", ret);
        return 0;
    }

    /* 💡 核心新增：让高通滤波器飞一会儿（等待 100ms 彻底滤除重力分量） */
    k_msleep(500);

    /* 💡 核心新增：再清空一次可能残留的初始中断 */
    lis3dh_clear_interrupt(&spi_dev, &int_src);

    /* 随后再配置并开启 STM32 的 GPIO 中断监听 */
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);

    /* 核心修改：增加 GPIO_PULL_DOWN，防止引脚浮空和断线时误触发 */
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | GPIO_PULL_DOWN);

    /* 保持不变, 在收到高电平时, 触发一个硬件中断 */
    // gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    // 设置中断回调函数
    // gpio_init_callback(&int1_cb_data, int1_gpio_isr, BIT(int1_gpio.pin));
    // gpio_add_callback(int1_gpio.port, &int1_cb_data);
    //

    printk("System Silent. STM32 is sleeping... Tap/Move the board now!\n");
    //
    printf("LIS3DH 硬件初始化成功，进入计步核心监测循环...\n");

    /* 4. 实例化并初始化纯整数计步器运行上下文
     * 参数含义：上下文指针, 量程(2代表±2g), 波动阈值(300 LSB), 迈步最小安全间隔(300ms)
     */
    walk_pedometer_t my_pedometer;
    walk_pedometer_init(&my_pedometer, 2, 300, 300);

    /* 5. 刷新高通滤波器，清空可能存在的前置历史干扰 */
    lis3dh_reset_baseline(&spi_dev);

    /* 6. 核心轮询采样与计步状态机处理 */
    while (1)
    {
        /* 使用给定的头文件 API 直接读取三轴 16 位原始 ADC 寄存器值 */
        ret = lis3dh_read_xyz(&spi_dev, &x_raw, &y_raw, &z_raw);
        if (ret == 0)
        {
            /* 获取符合 C17 约束的 Zephyr 系统毫秒时间戳 */
            int64_t now_ms = k_uptime_get();

            /* 调用纯整数极致加速算法：内部执行快速开方与数据平滑处理 */
            /* 之前：if (walk_pedometer_process(&my_pedometer, &x_raw, &y_raw, &z_raw, now_ms)) */
            /* 现在：直接传值，更快更安全 */
            if (walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms))
            {
                total_steps++;
                printf("[SPI WALK] 步数 +1！当前总数: %u\n", total_steps);
            }
        }
        else
        {
            printf("警告: 从 LIS3DH 读取三轴加速度数据失败 (错误码: %d)\n", ret);
        }

        /* 维持 25Hz 的高定时精度 */
        k_msleep(SAMPLING_RATE_MS);
    }

    return 0;
}
