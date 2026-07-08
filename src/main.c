#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>

#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases (led_user/button_user) missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);

/* 定义注册中断回调函数所需的结构体 */
static struct gpio_callback button_cb_data;

/*
 * 中断服务程序 (ISR) / 回调函数
 * 注意：中断函数内的代码要尽可能短，绝对不能在里面放复杂的延时或长循环
 */
void button_pressed_isr(const struct device *port, struct gpio_callback *cb,
                        gpio_port_pins_t pins)
{
    /* 在中断上下文中，直接安全读取按键的当前物理电平 */
    int btn_pressed = gpio_pin_get_dt(&btn);

    if (btn_pressed >= 0) {
        /* 让 PC7 灯实时跟随按键状态 */
        gpio_pin_set_dt(&led, btn_pressed);

        if (btn_pressed) {
            printk("[INTERRUPT] PC6 按下！(Uptime: %" PRId64 " ms)\n", k_uptime_get());
        } else {
            printk("[INTERRUPT] PC6 释放！\n");
        }
    }
}

int main(void)
{
    int ret;

    printk("--- ABrobot STM32F103RCT6 GPIO 硬件中断测试 ---\n");
    printk("Zephyr OS Version: %s (Interrupt-Driven Mode)\n", KERNEL_VERSION_STRING);

    /* 1. 初始化 LED */
    if (!gpio_is_ready_dt(&led)) {
        printk("CRITICAL: LED PC7 device not ready\n");
        return -1;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return -1;

    /* 2. 初始化按键引脚为输入模式 */
    if (!gpio_is_ready_dt(&btn)) {
        printk("CRITICAL: Button PC6 device not ready\n");
        return -1;
    }
    ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if (ret < 0) return -1;

    /*
     * 3. 配置引脚的硬件中断触发条件
     * GPIO_INT_EDGE_BOTH：代表按下（下降沿）和释放（上升沿）都会瞬间触发硬件中断
     */
    ret = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        printk("Error: Failed to configure pin interrupt (err %d)\n", ret);
        return -1;
    }

    /* 4. 初始化回调结构体，绑定我们的按键引脚编号与中断服务函数 */
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(btn.pin));

    /* 5. 将回调正式挂载到 GPIO 端口的管理子系统中 */
    ret = gpio_add_callback(btn.port, &button_cb_data);
    if (ret < 0) {
        printk("Error: Failed to register callback structure (err %d)\n", ret);
        return -1;
    }

    printk("硬件中断配置完毕！主线程即将进入无限期休眠，完全由 EXTI 硬件接管！\n");

    /*
     * 此时主循环不需要做任何轮询。
     * Zephyr 内核的空闲线程会自动将 CPU 核心推入低功耗模式。
     */
    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}
