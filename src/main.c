#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>

/*
 * 核心修正：设备树别名 'button-user' 在 C 代码宏中
 * 必须写为 'button_user'，否则破折号会被编译器当做减号处理。
 */
#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases (led_user/button_user) missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);

int main(void)
{
    int ret;

    printk("--- ABrobot STM32F103RCT6 IO Interaction Test ---\n");
    printk("Zephyr OS Version: %s (Boards Directory Overlay Mode)\n", KERNEL_VERSION_STRING);

    /* 验证 LED 外设 */
    if (!gpio_is_ready_dt(&led)) {
        printk("CRITICAL: LED PC7 device not ready\n");
        return -1;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return -1;

    /* 验证按键外设 */
    if (!gpio_is_ready_dt(&btn)) {
        printk("CRITICAL: Button PC6 device not ready\n");
        return -1;
    }
    ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if (ret < 0) return -1;

    printk("System ready! Press 'PC6' button to light up 'PC7' LED.\n");

    int last_state = -1;

    while (1) {
        /* 读取按键电平（由于 ACTIVE_LOW 属性，按下返回 1，松开返回 0） */
        int btn_pressed = gpio_pin_get_dt(&btn);

        if (btn_pressed < 0) {
            printk("ERROR: Failed to read button\n");
            k_msleep(100);
            continue;
        }

        /* 驱动 LED 灯状态跟随按键 */
        gpio_pin_set_dt(&led, btn_pressed);

        /* 状态发生改变时触发串口事件报告 */
        if (btn_pressed != last_state) {
            if (btn_pressed) {
                printk("[EVENT] Button PC6 PRESSED -> LED PC7 ON! (Uptime: %" PRId64 " ms)\n", k_uptime_get());
            } else {
                printk("[EVENT] Button PC6 RELEASED -> LED PC7 OFF!\n");
            }
            last_state = btn_pressed;
        }

        /* 20ms 频率循环（兼顾软件防抖） */
        k_msleep(20);
    }
    return 0;
}
