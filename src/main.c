#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>  /*  加入这行，用以支持 KERNEL_VERSION_STRING 宏 */

/* Zephyr 4.4 推荐使用标准的 C17 宏展开检查设备树节点是否存在 */
#if !DT_NODE_EXISTS(DT_ALIAS(led_test))
#error "Error: Device tree alias 'led-test' is required by this application!"
#endif

#define LED_NODE DT_ALIAS(led_test)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

int main(void)
{
    int ret;

    printk("--- ABrobot STM32F103RCT6 Mini Board Verification ---\n");
    /* 打印 Zephyr 4.4.1 当前系统的版本信息 */
    printk("Zephyr OS Version: %s (C17 Compiler Standard Ready)\n", KERNEL_VERSION_STRING);

    /* 检查硬件设备是否就绪 */
    if (!gpio_is_ready_dt(&led)) {
        printk("CRITICAL: GPIO device for LED is not ready\n");
        return -1;
    }

    /* 配置引脚 */
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("CRITICAL: Failed to configure GPIO (error: %d)\n", ret);
        return -1;
    }

    printk("Hardware check passed. System loop entered.\n");

    while (1) {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            printk("ERROR: GPIO toggle failed\n");
            break;
        }

        /* 4.4 内核常驻内存，此处的 uptime 计数可作为精准的时钟源指标 */
        int64_t current_uptime = k_uptime_get();
        printk("[ALIVE] System Uptime: %" PRId64 " ms\n", current_uptime);

        /* 500ms 翻转，由于 Zephyr 4.4 全面强化了低功耗唤醒，无任务时 CPU 将直接挂起 */
        k_msleep(500);
    }
    return 0;
}
