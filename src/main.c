#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "lis3dh.h"

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
    if (ret < 0) {
        printk("Sensor init failed: %d\n", ret);
        return 0;
    }

    // 2. 核心关键：让子弹飞一会儿！静止等待 200ms，让高通滤波器彻底“吃掉”并稳定地球重力
    k_msleep(200);

    uint8_t who_am_i = 0;
   ret =  lis3dh_reg_read(&spi_dev, 0x0F, &who_am_i, 1);

    if (ret < 0) {
        printk("LIS3DSH WHO_AM_I read failed: %d\n", ret);
        return ret;
    } else
    {
        printk("LIS3DSH WHO_AM_I: 0x%02X\n", who_am_i);
    }

    // 3. 稳准狠：此时传感器已完全稳定，执行校准，扣除重力基准并强行释放 INT1 高电平
    ret = lis3dh_reset_baseline(&spi_dev);
    if (ret < 0) {
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
    gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    // 设置中断回调函数
    gpio_init_callback(&int1_cb_data, int1_gpio_isr, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);
    //

    printk("System Silent. STM32 is sleeping... Tap/Move the board now!\n");

    int count = 0;
    while (1)
    {
        /* STM32 在此内核信号量处完全挂起休眠，0% CPU 占用 */
        k_sem_take(&motion_sem, K_FOREVER);

        /* 瞬间清空中断锁存，允许下一次中断触发 */
        lis3dh_clear_interrupt(&spi_dev, &int_src);
        k_msleep(800);
        /* 触发时仅打印单行日志 */
        printk("%d Sensor woke up STM32. (Interrupt Source: 0x%02X)\n", count,  int_src);

        /* 800ms 防抖，防止手拿放过程中连续弹出一堆日志 */
        k_msleep(800);
        count ++;
    }
}
