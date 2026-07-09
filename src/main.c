#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "lis3dh.h"

#define LIS3DH_NODE DT_NODELABEL(lis3dsh)

/* 声明 SPI 配置 */
static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DH_NODE,
                                          SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
                                          0);

/* 从设备树自动获取 irq-gpios 对应的引脚参数 */
// static const struct gpio_dt_spec int1_gpio = GPIO_DT_SPEC_GET(LIS3DH_NODE, irq_gpios);

/* 从 zephyr,user 节点中抓取这个自定义属性 */
static const struct gpio_dt_spec int1_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensor_irq_gpios);

/* 初始化一个初始值为 0 的信号量 */
K_SEM_DEFINE(motion_sem, 0, 1);

static struct gpio_callback int1_cb_data;

/* GPIO 中断回调函数（运行于 ISR 紧急上下文） */
void int1_gpio_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* 极速操作：只给信号量，通知 main 线程起床干活 */
    k_sem_give(&motion_sem);
}

int main(void)
{
    int ret;
    uint8_t int_src = 0;

    printk("--- LIS3DH Motion Interrupt Refactor (Zephyr 4.4.1) ---\n");

    if (!spi_is_ready_dt(&spi_dev) || !gpio_is_ready_dt(&int1_gpio)) {
        printk("Error: Peripherals not ready!\n");
        return 0;
    }

    /* 初始化 LIS3DH 的中断寄存器组 */
    ret = lis3dh_init(&spi_dev);
    if (ret < 0) {
        printk("Sensor init failed: %d\n", ret);
        return 0;
    }
    printk("Sensor configured. Configured STM32 GPIO interrupt...\n");

    /* 配置 STM32 的中断输入引脚 */
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT);
    /* 默认高电平有效，所以配置为边缘触发至有效电平（上升沿） */
    gpio_pin_interrupt_configure_dt(&int1_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    /* 绑定 ISR */
    gpio_init_callback(&int1_cb_data, int1_gpio_isr, BIT(int1_gpio.pin));
    gpio_add_callback(int1_gpio.port, &int1_cb_data);

    printk("System Ready! Shaking the board to trigger print...\n");

    while (1) {
        /* 线程在此处挂起休眠，完全不消耗 CPU，直到被 ISR 唤醒 */
        k_sem_take(&motion_sem, K_FOREVER);

        printk("[🔥 Wakeup] Motion Detected! ");

        /* 必须在线程中读取并清除中断，否则 INT1 引脚会一直维持高电平，无法产生下次沿变 */
        ret = lis3dh_clear_interrupt(&spi_dev, &int_src);
        if (ret == 0) {
            printk("(Source Reg: 0x%02X)\n", int_src);
        }

        /* 顺便打印一下触发时的即时数据 */
        uint8_t axis_data[6];
        if (lis3dh_reg_read(&spi_dev, LIS3DH_REG_OUT_X_L, axis_data, 6) == 0) {
            int16_t x = (int16_t)((axis_data[1] << 8) | axis_data[0]);
            int16_t y = (int16_t)((axis_data[3] << 8) | axis_data[2]);
            int16_t z = (int16_t)((axis_data[5] << 8) | axis_data[4]);
            printk("   Accel Data -> X: %d | Y: %d | Z: %d\n", x, y, z);
        }

        /* 故意延时 300ms 充当软件防抖，防止高频震动时串口打印雪崩 */
        k_msleep(300);
    }
}
