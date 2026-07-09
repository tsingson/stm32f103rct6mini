#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>     /* Zephyr 标准显示屏抽象层 */
#include <zephyr/display/cfb.h>          /* 标准字符帧缓冲区 */
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/version.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>            /* 引入现代策略锁 API */
#include <stdio.h>
#include <string.h>

#if !DT_NODE_EXISTS(DT_ALIAS(led_user)) || !DT_NODE_EXISTS(DT_ALIAS(button_user))
#error "Error: Critical device tree aliases missing!"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led_user), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_user), gpios);
static struct gpio_callback button_cb_data;

static const struct device* display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

K_SEM_DEFINE(sleep_countdown_sem, 0, 1);

enum tracker_state
{
    STATE_0_HEARTBEAT = 0,
    STATE_1_TRACKING = 1,
    STATE_3_EMERGENCY = 3,
    STATE_4_BATTERY_REP = 4
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

/* ==================== 屏幕UI数据刷新显示函数 ==================== */
void update_oled_ui(void)
{
    if (!device_is_ready(display_dev))
    {
        return;
    }

    char buf[32]; /* 修正：定义为标准本地栈缓冲区字符数组 */

    cfb_framebuffer_clear(display_dev, false);

    cfb_draw_text(display_dev, "IOT TRACKER v4.4", 0, 0);

    snprintf(buf, sizeof(buf), "MODE STATE: [%d]", (int)current_fsm_state);
    cfb_draw_text(display_dev, buf, 0, 18);

    if (gpio_pin_get_dt(&btn) == 1)
    {
        cfb_draw_text(display_dev, "STATUS: ACTIVE", 0, 36);
    }
    else
    {
        cfb_draw_text(display_dev, "STATUS: IDLE_WAIT", 0, 36);
    }

    cfb_framebuffer_finalize(display_dev);
}

/* =============================================================================== */

static int64_t last_interrupt_time = 0;
#define DEBOUNCE_DELAY_MS  30

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
    safe_log("--- ABrobot STM32F103RCT6 Async OLED Display Test ---\n");

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&btn))
    {
        return -1;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&btn, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &button_cb_data);

    if (!device_is_ready(display_dev))
    {
        safe_log("CRITICAL: SSD1306 OLED display device not ready via I2C1\n");
    }
    else
    {
        cfb_framebuffer_init(display_dev);

        uint8_t num_fonts = cfb_get_numof_fonts(display_dev);
        for (uint8_t i = 0; i < num_fonts; i++)
        {
            /* 核心修正：严格对齐 Zephyr 4.4.1 的 uint8_t 宽高数据类型定义 */
            uint8_t width, height;
            cfb_get_font_size(display_dev, i, &width, &height);
            safe_log("Loaded Built-in Font index %d: Size %dx%d\n", (int)i, (int)width, (int)height);
        }

        cfb_set_kerning(display_dev, 0);
        safe_log("OLED Driver initialization sequence passed.\n");

        update_oled_ui();
    }

    while (1)
    {
        update_oled_ui();

        k_sem_take(&sleep_countdown_sem, K_FOREVER);

        safe_log("松开检测成功，系统将在 2 秒后进入微安级 Deep Sleep...\n");
        update_oled_ui();
        k_msleep(2000);

        if (gpio_pin_get_dt(&btn) == 0)
        {
            safe_log("\n⚠️-----------------[ CRITICAL: PRE-SLEEP WARNING ]-----------------⚠️\n");
            safe_log("[🚨 断电提醒] 即将进入微安级深度睡眠(Stop Mode)！\n");

            if (device_is_ready(display_dev))
            {
                display_blanking_on(display_dev);
            }

            while (!ring_buf_is_empty(&log_ring_buf))
            {
                k_yield();
            }
            k_msleep(5);

            /*
             * ==================== Zephyr 4.4.1 现代策略控制休眠法 ====================
             * 废弃旧的强制接口，改为通过策略锁（Policy Lock）控制内核：
             * 我们锁定所有的浅度睡眠等级（比如 RUNTIME_IDLE），强制告诉内核在 WFI 时
             * 必须滑入最深也最省电的 SUSPEND_TO_RAM (Stop 模式)。
             */
            /*
 * ==================== Zephyr 4.4.1 现代策略控制休眠法 ====================
 * 修正：使用 Zephyr 4.4.1 官方标准的 PM_ALL_SUBSTATES 宏
 */
            pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
            /* 执行内核级汇编指令，芯片正式断电沉睡... */
            k_cpu_idle();


            /* =======================================================================
                * ⚡ 硬件苏醒线（显式、稳健、最高优先级的复苏段落）
                * ======================================================================= */

            /* 修正：解锁时同样对齐使用 PM_ALL_SUBSTATES */
            pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);

            SET_BIT(RCC->CR, RCC_CR_HSION);
            while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0);

#if CONFIG_TRACING
            sys_trace_idle_exit();
#endif
            if (device_is_ready(display_dev))
            {
                display_blanking_off(display_dev);
            }

            safe_log("\n⚡-----------------[ CRITICAL: WAKEUP DETECTED ]-----------------⚡\n");
            safe_log("[📢 唤醒提醒] 检测到 PC6 外部中断！系统时钟和显示外设已硬恢复就绪！\n");

            update_oled_ui();
        }
    }
    return 0;
}
