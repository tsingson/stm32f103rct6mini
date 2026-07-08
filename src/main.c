#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>

#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases (led_user/button_user) missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);

static struct gpio_callback button_cb_data;

/*
 * 中断防抖核心变量
 * 使用 int64_t 记录上一次中断发生的系统物理时间（毫秒级）
 */
static int64_t last_interrupt_time = 0;
#define DEBOUNCE_DELAY_MS  30  /* 30毫秒防抖窗口，过滤所有机械弹跳 */

void button_pressed_isr(const struct device *port, struct gpio_callback *cb,
                        gpio_port_pins_t pins)
{
    int64_t current_time = k_uptime_get();

    /*
     * 核心算法：检查当前时间与上一次有效中断时间的差值
     * 如果小于 30 毫秒，说明是金属弹片在颤动，直接退出（丢弃本次伪信号）
     */
    if ((current_time - last_interrupt_time) < DEBOUNCE_DELAY_MS) {
        return;
    }

    int btn_pressed = gpio_pin_get_dt(&btn);
    if (btn_pressed >= 0) {
        gpio_pin_set_dt(&led, btn_pressed);

        if (btn_pressed) {
            printk("[DEBOUNCED ISR] PC6 稳态按下！(Uptime: %" PRId64 " ms)\n", current_time);
        } else {
            printk("[DEBOUNCED ISR] PC6 稳态释放！\n");
        }

        /* 只有成功通过防抖校验的有效事件，才更新时间戳基准 */
        last_interrupt_time = current_time;
    }
}

int main(void)
{
    int ret;

    printk("--- ABrobot STM32F103RCT6 Debounced Interrupt Test ---\n");
    printk("Zephyr OS Version: %s (High-Precision Anti-Bounce Enabled)\n", KERNEL_VERSION_STRING);

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn)) {
        printk("CRITICAL: Hardware peripherals not ready\n");
        return -1;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    /* 配置中断：双沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    if (ret < 0) return -1;

    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(btn.pin));
    ret = gpio_add_callback(btn.port, &button_cb_data);
    if (ret < 0) return -1;

    printk("消抖版中断就绪。你可以尝试疯狂快速连按，观察过滤效果：\n");

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}
