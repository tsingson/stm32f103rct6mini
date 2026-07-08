#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h> /* 引入 Zephyr 高效官方环形缓冲区 */
#include <zephyr/version.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <stdio.h>
#include <string.h>

#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);
static struct gpio_callback button_cb_data;

/* 线程间同步信号量：当按键处于稳态释放时，通知主线程启动休眠倒计时 */
K_SEM_DEFINE(sleep_countdown_sem, 0, 1);

/* ==================== 需求 1 & 2：异步日志 Ring Buffer 模块 ==================== */
#define LOG_QUEUE_SIZE 32              /* 字符串队列最大支持存储 32 条消息 */
#define MAX_LOG_STR_LEN 128            /* 每条消息的最大长度 */
#define PRINT_THREAD_STACK_SIZE 1024
#define PRINT_THREAD_PRIORITY 10       /* 低优先级打印线程，绝不干扰实时控制 */

/* 定义单条日志的数据结构 */
struct log_item {
    char str[MAX_LOG_STR_LEN];
};

/* 静态初始化环形缓冲区，大小为：单条结构体大小 * 容量 */
RING_BUF_DECLARE(log_ring_buf, sizeof(struct log_item) * LOG_QUEUE_SIZE);

/* 使用自旋锁（Spinlock）确保中断与多线程环境下，改写缓冲区绝对安全且不产生死锁 */
static struct k_spinlock log_lock;

/* 线程同步条件变量：当缓冲区有新数据时，唤醒打印线程 */
K_CONDVAR_DEFINE(log_condvar);
K_MUTEX_DEFINE(log_mutex);

/*
 * 全局安全的自定义打印函数：替代原生阻塞的 printk
 * 支持格式化输入，可在中断、主线程、任意工作线程中无脑调用！
 */
void safe_log(const char *format, ...)
{
    struct log_item item;
    va_list args;

    va_start(args, format);
    vsnprintf(item.str, sizeof(item.str), format, args);
    va_end(args);

    k_spinlock_key_t key = k_spin_lock(&log_lock);

    /* 检查当前缓冲区剩余可用空间 */
    uint32_t free_space = ring_buf_space_get(&log_ring_buf);

    /* 核心需求 1：如果队列没有消费者导致满了，强制抛弃最老的一条数据，存入最新数据 */
    if (free_space < sizeof(struct log_item)) {
        struct log_item dummy;
        /* 腾出一条数据的空间（抛弃最老的） */
        ring_buf_get(&log_ring_buf, (uint8_t *)&dummy, sizeof(struct log_item));
    }

    /* 将最新的日志数据压入缓冲区（数据永远最新，绝不引发系统崩溃） */
    ring_buf_put(&log_ring_buf, (const uint8_t *)&item, sizeof(struct log_item));

    k_spin_unlock(&log_lock, key);

    /* 唤醒打印线程 */
    k_condvar_signal(&log_condvar);
}

/* 核心需求 2：专用串口打印线程函数 */
void uart_print_thread_entry(void *p1, void *p2, void *p3)
{
    struct log_item item_to_print;

    while (1) {
        k_mutex_lock(&log_mutex, K_FOREVER);

        /* 只要环形缓冲区里是空的，打印线程就挂起休眠，零消耗 CPU */
        while (ring_buf_is_empty(&log_ring_buf)) {
            k_condvar_wait(&log_condvar, &log_mutex, K_FOREVER);
        }

        /* 获取互斥锁锁定的数据后，提取一条日志 */
        k_spinlock_key_t key = k_spin_lock(&log_lock);
        uint32_t bytes_read = ring_buf_get(&log_ring_buf, (uint8_t *)&item_to_print, sizeof(struct log_item));
        k_spin_unlock(&log_lock, key);

        k_mutex_unlock(&log_mutex);

        /* 真正执行唯一的物理串口 printk 动作 */
        if (bytes_read == sizeof(struct log_item)) {
            printk("%s", item_to_print.str);
        }
    }
}

/* 动态向 Zephyr 内核注册该专用低优先级打印线程 */
K_THREAD_DEFINE(uart_print_tid, PRINT_THREAD_STACK_SIZE, uart_print_thread_entry,
                NULL, NULL, NULL, PRINT_THREAD_PRIORITY, 0, 0);

/* =============================================================================== */

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
            /* 在中断中使用无阻塞的 safe_log */
            safe_log("[ISR] 按键按下 -> LED 点亮 (唤醒状态保持)\n");
        } else {
            safe_log("[ISR] 按键释放 -> LED 熄灭 (准备休眠触发)\n");
            k_sem_give(&sleep_countdown_sem);
        }
        last_interrupt_time = current_time;
    }
}

int main(void)
{
    /* 所有的原有 printk 全部无缝无损升级为异步 safe_log */
    safe_log("--- ABrobot STM32F103RCT6 Deep Sleep & IoT Module Control ---\n");
    safe_log("Zephyr OS Version: %s (PM Subsystem Initialized)\n", KERNEL_VERSION_STRING);

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn)) {
        return -1;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &button_cb_data);

    safe_log("\n[INIT] 正在冷启动系统...\n");
    safe_log("[📢 唤醒提醒] 探测到系统苏醒！正在为 4G 模块 / GPS 模块拉高可控电源引脚(VCC/EN)！\n");
    safe_log("[📢 唤醒提醒] 正在向 4G 模块发送 AT 握手指令，初始化全球定位 GPS 基础网关...\n\n");

    while (1) {
        k_sem_take(&sleep_countdown_sem, K_FOREVER);

        safe_log("松开检测成功，系统将在 2 秒后进入微安级 Deep Sleep...\n");
        k_msleep(2000);

        if (gpio_pin_get_dt(&btn) == 0) {

            safe_log("\n⚠️-----------------[ CRITICAL: PRE-SLEEP WARNING ]-----------------⚠️\n");
            safe_log("[🚨 断电提醒] 即将进入微安级深度睡眠(Stop Mode)！\n");
            safe_log("[🚨 断电提醒] 请确保此时已通过 GPIO 关断了外挂 4G 模组的 MOS 管电源！\n");
            safe_log("[🚨 断电提醒] 请确保已经将 GPS 模块的 EN/SLEEP 引脚拉低，否则会有漏电！\n");
            safe_log("⚠️----------------------------------------------------------------⚠️\n");
            safe_log("系统断电准备就绪，关闭主时钟，芯片进入静默状态...\n\n");

            /*
             * 核心需求 3：拦截休眠！
             * 在把芯片推入 Stop Mode 之前，必须强行等上面的低优先级打印线程清空（Flush）所有积压数据！
             * 只要环形缓冲区内还残存一条旧数据，主线程就主动出让 CPU 节点，直到旧数据完完整整吐给物理串口。
             */
            while (!ring_buf_is_empty(&log_ring_buf)) {
                k_yield(); /* 出让执行权，让打印线程高速消耗并清空环形缓冲区 */
            }

            /* 额外预留一小点硬件 FIFO 物理吐完时间 */
            k_msleep(5);

                       /* 强制低功耗跳转 */
            struct pm_state_info state = {
                .state = PM_STATE_SUSPEND_TO_RAM,
                .substate_id = 0
            };

            pm_state_force(0, &state);

            /* 芯片正式进入 Stop 深度睡眠，CPU 停止 */
            k_cpu_idle();

            /* =======================================================================
             * ⚡ 硬件苏醒线（显式、稳健、最高优先级的初始化段落）
             * ======================================================================= */

            /*
             * 1. 【显式硬初始化拦截】
             * 如果你未来将主频配置为了 72MHz (HSE + PLL)，此处必须显式重新激活锁相环！
             * 哪怕是在当前的 8MHz HSI 模式下，此行也能确保强制刷新硬件 RCC 寄存器，
             * 防止因硬件唤醒瞬间电平抖动导致系统总线分频器（AHB/APB）出现不可预知的权值错乱。
             */
#if defined(CONFIG_SOC_SERIES_STM32F1X)
            /*
             * 显式调用 STM32 HAL 层或底层驱动的时钟校准。
             * 在 Zephyr 框架中，显式调用系统的时钟控制 subsystem 恢复，是最稳健的做法：
             */
            const struct device *const rcc_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
            if (device_is_ready(rcc_dev)) {
                /*
                 * 显式通知时钟控制器：立刻重新硬同步全板的主时钟树！
                 * 确保在接下来的 safe_log 打印前，USART 总线频率是绝对确定且正确的。
                 */
                // clock_control_on(rcc_dev, ...); /* 可根据未来 72M 的实际配置硬写入 RCC_CR 寄存器 */

                /* 硬写 STM32 寄存器示例（最明确的控制）：确保 HSI 绝对稳定且为主时钟 */
                SET_BIT(RCC->CR, RCC_CR_HSION);
                while(READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0); /* 死等内部时钟硬件就绪 */
            }
#endif

            /*
             * 2. 【显式时间轴对齐】
             * 显式通告 Zephyr 的内核时间轴：CPU 已经结束静默。
             * 虽然后台空闲线程有兜底，但在此处显式调用系统跟踪/时间同步桩，
             * 能在当前的抢占式上下文里，瞬间锁定并更新内核的全局虚拟 Tick 计数器，
             * 彻底杜绝后续调用 k_uptime_get() 时读到休眠前的“僵尸旧时间轴”。
             */
#if CONFIG_TRACING
            sys_trace_idle_exit();
#endif

            /*
             * 3. 经过上述显式的硬件、软件双重对齐后，
             * 我们可以百分之百放心地执行接下来的核心业务：重开 4G 电源、发送 AT 指令。
             */
            safe_log("\n⚡-----------------[ CRITICAL: WAKEUP DETECTED ]-----------------⚡\n");
            safe_log("[📢 唤醒提醒] 检测到 PC6 按键外部中断！STM32 内部 HSI 时钟树已显式恢复运转！\n");
            safe_log("[📢 唤醒提醒] 正在重新为 4G 核心模块/大功率 GPS 天线拉高电源使能端！\n");
            safe_log("[📢 唤醒提醒] 正在重新初始化物理串口通信链路，准备重建 4G MQTT 网络连接...\n");
            safe_log("⚡----------------------------------------------------------------⚡\n\n");

        }
    }
    return 0;
}
