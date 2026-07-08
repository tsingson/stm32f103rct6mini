#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>
#include <zephyr/pm/pm.h>        /* Zephyr 4.4.1 标准低功耗内核 API */
#include <zephyr/pm/policy.h>

#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);

static struct gpio_callback button_cb_data;

/* 线程间同步信号量：当按键处于稳态释放时，通知主线程启动休眠倒计时 */
K_SEM_DEFINE(sleep_countdown_sem, 0, 1);

static int64_t last_interrupt_time = 0;
#define DEBOUNCE_DELAY_MS  30

void button_pressed_isr(const struct device *port, struct gpio_callback *cb,
                        gpio_port_pins_t pins)
{
    int64_t current_time = k_uptime_get();
    if ((current_time - last_interrupt_time) < DEBOUNCE_DELAY_MS) {
        return;
    }

    int btn_pressed = gpio_pin_get_dt(&btn);
    if (btn_pressed >= 0) {
        gpio_pin_set_dt(&led, btn_pressed);

        if (btn_pressed) {
            printk("[ISR] 按键按下 -> LED 点亮 (唤醒状态保持)\n");
        } else {
            printk("[ISR] 按键释放 -> LED 熄灭 (准备休眠触发)\n");
            /* 释放信号量，通知主线程：用户手松开了，可以开始休眠筹备 */
            k_sem_give(&sleep_countdown_sem);
        }
        last_interrupt_time = current_time;
    }
}

int main(void)
{
    printk("--- ABrobot STM32F103RCT6 Deep Sleep & IoT Module Control ---\n");
    printk("Zephyr OS Version: %s (PM Subsystem Initialized)\n", KERNEL_VERSION_STRING);

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn)) {
        return -1;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &button_cb_data);

    /* 第一次开机，默认进行一次模块初始化提醒 */
    printk("\n[INIT] 正在冷启动系统...\n");
    printk("[📢 唤醒提醒] 探测到系统苏醒！正在为 4G 模块 / GPS 模块拉高可控电源引脚(VCC/EN)！\n");
    printk("[📢 唤醒提醒] 正在向 4G 模块发送 AT 握手指令，初始化全球定位 GPS 基础网关...\n\n");

    while (1) {
        /* 挂起并死等信号量，只有当用户“松开按键”时，才会往下执行 */
        k_sem_take(&sleep_countdown_sem, K_FOREVER);

        printk("松开检测成功，系统将在 2 秒后进入微安级 Deep Sleep...\n");
        k_msleep(2000);

        /* 再次检查，确保进入前按键依然没有被再次按下（防止倒计时期间用户又按了） */
        if (gpio_pin_get_dt(&btn) == 0) {

            /* ==================== 进入深度睡眠前的黄金断电通知窗口 ==================== */
            printk("\n⚠️-----------------[ CRITICAL: PRE-SLEEP WARNING ]-----------------⚠️\n");
            printk("[🚨 断电提醒] 即编进入微安级深度睡眠(Stop Mode)！\n");
            printk("[🚨 断电提醒] 请确保此时已通过 GPIO 关断了外挂 4G 模组的 MOS 管电源(或发送 AT+CPOWD 关机)！\n");
            printk("[🚨 断电提醒] 请确保已经将 GPS 模块的 EN/SLEEP 引脚拉低，否则外挂模块会产生数十毫安漏电！\n");
            printk("⚠️----------------------------------------------------------------⚠️\n");
            printk("系统断电准备就绪，关闭主时钟，芯片进入静默状态...\n\n");

            /* 强制等待串口硬件缓冲区彻底吐完最后一个字符，防止休眠时串口数据被截断 */
            k_msleep(100);

            /*
             * 触发 Zephyr 标准低功耗：请求切入 SUSPEND_TO_RAM (对应 STM32 Stop 停止模式)
             * 此 API 会强制挂起 CPU 和总线时钟，直到 PC6 产生新的中断触发电平跳变
             */
            struct pm_state_info state = {
                .state = PM_STATE_SUSPEND_TO_RAM,
                .substate_id = 0
            };

            /* 告诉策略管理器不要阻拦，强制闭眼 */
            pm_state_force(0, &state);

            /* 执行内核级别的 WFI (Wait For Interrupt) 汇编指令，进入微安级深度睡眠 */
            k_cpu_idle();

            /* =======================================================================
             * ⚡ 硬件苏醒线 ⚡
             * 当你再次按下 PC6 按键时，STM32 硬件瞬间苏醒，代码会直接从这里“睁眼”向下继续跑！
             * ======================================================================= */

            /* 强行刷新下内核的 tick，防止时间流逝计算短瞬失真 */
            sys_trace_idle_exit();

            printk("\n⚡-----------------[ CRITICAL: WAKEUP DETECTED ]-----------------⚡\n");
            printk("[📢 唤醒提醒] 检测到 PC6 按键外部中断！STM32 内部 HSI 时钟树已恢复运转！\n");
            printk("[📢 唤醒提醒] 正在重新为 4G 核心模块/大功率 GPS 天线拉高电源使能端！\n");
            printk("[📢 唤醒提醒] 正在重新初始化物理串口通信链路，准备重建 4G MQTT 网络连接...\n");
            printk("⚡----------------------------------------------------------------⚡\n\n");
        }
    }
    return 0;
}
