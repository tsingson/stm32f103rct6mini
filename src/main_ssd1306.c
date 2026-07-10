/* src/main.c */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/version.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <stdio.h>
#include <string.h>

#include "ssd1306_app.h" /* 引入新拆分的显示子模块头文件 */

#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);
static struct gpio_callback button_cb_data;

K_SEM_DEFINE(sleep_countdown_sem, 0, 1);

enum tracker_state
{
    STATE_0_HEARTBEAT = 0,
    STATE_1_TRACKING = 1
};

static enum tracker_state current_fsm_state = STATE_0_HEARTBEAT;

/* ==================== 异步日志 Ring Buffer 模块 ==================== */
#define LOG_QUEUE_SIZE 32
#define MAX_LOG_STR_LEN 128
#define PRINT_THREAD_STACK_SIZE 1024
#define PRINT_THREAD_PRIORITY 10

struct log_item
{
    char str[MAX_LOG_STR_LEN];
};

RING_BUF_DECLARE(log_ring_buf, sizeof(struct log_item) * LOG_QUEUE_SIZE);
static struct k_spinlock log_lock;
K_CONDVAR_DEFINE(log_condvar);
K_MUTEX_DEFINE(log_mutex);

void safe_log(const char* format, ...)
{
    struct log_item item;
    va_list args;
    va_start(args, format);
    vsnprintf(item.str, sizeof(item.str), format, args);
    va_end(args);

    k_spinlock_key_t key = k_spin_lock(&log_lock);
    uint32_t free_space = ring_buf_space_get(&log_ring_buf);
    if (free_space < sizeof(struct log_item))
    {
        struct log_item dummy;
        ring_buf_get(&log_ring_buf, (uint8_t*)&dummy, sizeof(struct log_item));
    }
    ring_buf_put(&log_ring_buf, (const uint8_t*)&item, sizeof(struct log_item));
    k_spin_unlock(&log_lock, key);
    k_condvar_signal(&log_condvar);
}

void uart_print_thread_entry(void* p1, void* p2, void* p3)
{
    struct log_item item_to_print;
    while (1)
    {
        k_mutex_lock(&log_mutex, K_FOREVER);
        while (ring_buf_is_empty(&log_ring_buf))
        {
            k_condvar_wait(&log_condvar, &log_mutex, K_FOREVER);
        }
        k_spinlock_key_t key = k_spin_lock(&log_lock);
        uint32_t bytes_read = ring_buf_get(&log_ring_buf, (uint8_t*)&item_to_print, sizeof(struct log_item));
        k_spin_unlock(&log_lock, key);
        k_mutex_unlock(&log_mutex);

        if (bytes_read == sizeof(struct log_item))
        {
            printk("%s", item_to_print.str);
        }
    }
}

K_THREAD_DEFINE(uart_print_tid, PRINT_THREAD_STACK_SIZE, uart_print_thread_entry, NULL, NULL, NULL,
                PRINT_THREAD_PRIORITY, 0, 0);

/* ==================== 统一的非阻塞 UI 更新投递代理 ==================== */
void push_display_ui_update(void)
{
    struct display_msg_packet msg;
    memset(&msg, 0, sizeof(msg));

    /* 填充多行文本 */
    strncpy(msg.lines[0], "IOT v4.4", DISPLAY_LINE_MAX_LEN);
    snprintf(msg.lines[1], DISPLAY_LINE_MAX_LEN, "MODE: [%d]", (int)current_fsm_state);

    if (gpio_pin_get_dt(&btn) == 1)
    {
        strncpy(msg.lines[2], "S: ACTIVE", DISPLAY_LINE_MAX_LEN);
    }
    else
    {
        strncpy(msg.lines[2], "S: IDLE_WAIT", DISPLAY_LINE_MAX_LEN);
    }

    /*
     * 核心解耦：将打包好的数据无脑丢进消息队列。
     * K_NO_WAIT 代表如果队列由于极端原因满了，直接覆盖或报错退出，绝不在中断/业务中发生阻塞！
     */
    k_msgq_put(&display_msg_q, &msg, K_NO_WAIT);
}

/* =============================================================================== */

static int64_t last_interrupt_time = 0;
#define DEBOUNCE_DELAY_MS  3

void button_pressed_isr(const struct device* port, struct gpio_callback* cb,
                        gpio_port_pins_t pins)
{
    int64_t current_time = k_uptime_get();
    if ((current_time - last_interrupt_time) < DEBOUNCE_DELAY_MS)
    {
        return;
    }

    int btn_pressed = gpio_pin_get_dt(&btn);
    if (btn_pressed >= 0)
    {
        gpio_pin_set_dt(&led, btn_pressed);

        if (btn_pressed)
        {
            safe_log("[ISR] 按键按下 -> LED 点亮 (唤醒状态保持)\n");
            current_fsm_state = (current_fsm_state == STATE_0_HEARTBEAT) ? STATE_1_TRACKING : STATE_0_HEARTBEAT;

            /* 在中断上下文直接调用，非阻塞向队列派发 UI 数据 */
            push_display_ui_update();
        }
        else
        {
            safe_log("[ISR] 按键释放 -> LED 熄灭 (准备休眠触发)\n");
            k_sem_give(&sleep_countdown_sem);
        }
        last_interrupt_time = current_time;
    }
}

int main(void)
{
    safe_log("--- ABrobot STM32F103RCT6 Split Thread UI System ---\n");

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn))
    {
        return -1;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &button_cb_data);

    /* 开机主动刷新一次初始数据包到显示队列 */
    push_display_ui_update();

    while (1)
    {
        /* 主常态循环，如果有状态变更随时投递 */
        push_display_ui_update();

        k_sem_take(&sleep_countdown_sem, K_FOREVER);

        safe_log("松开检测成功，系统将在 2 秒后进入微安级 Deep Sleep...\n");
        push_display_ui_update();
        k_msleep(2000);

        if (gpio_pin_get_dt(&btn) == 0)
        {
            safe_log("\n⚠️-----------------[ CRITICAL: PRE-SLEEP WARNING ]-----------------⚠️\n");
            safe_log("[🚨 断电提醒] 即将进入微安级深度睡眠(Stop Mode)！\n");

            /*
             * 需求 3 的多维度拦截：
             * 1. 拦截一：必须等显示内核消息队列里的挂起包被完全消耗掉 (`num_used == 0`)
             * 2. 拦截二：必须等物理串口日志环形缓冲区被完全清空 (`is_empty == true`)
             */
            while ((k_msgq_num_used_get(&display_msg_q) > 0) || !ring_buf_is_empty(&log_ring_buf))
            {
                k_yield(); /* 疯狂挂起当前主任务，把时间切片借给显示线程和串口线程，直到它们全干完活 */
            }

            /* 最后一包数据渲染完毕，物理关断屏幕硬件电荷泵，彻底防漏电 */
            ssd1306_set_blanking(true);

            k_msleep(5);

            /* 现代策略控制休眠 */
            pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
            k_cpu_idle();

            /* =======================================================================
             * ⚡ 硬件苏醒线（显式、稳健、最高优先级的复苏段落）
             * ======================================================================= */
            pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);

            SET_BIT(RCC->CR, RCC_CR_HSION);
            while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0);

#if CONFIG_TRACING
            sys_trace_idle_exit();
#endif
            /* 硬件睁眼的第一微秒：物理拉通屏幕供电扫描线 */
            ssd1306_set_blanking(false);

            safe_log("\n⚡-----------------[ CRITICAL: WAKEUP DETECTED ]-----------------⚡\n");
            safe_log("[📢 唤醒提醒] 检测到 PC6 外部中断！多线程消息内核和显示外设同步复苏！\n");

            /* 唤醒后，无脑投递刷新一包全新复苏状态的 UI 报文 */
            push_display_ui_update();
        }
    }
    return 0;
}
