/* src/ssd1306_app.c */
#include "ssd1306_app.h"
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <stdio.h>
#include <string.h>

#define DISPLAY_THREAD_STACK_SIZE 1024
#define DISPLAY_THREAD_PRIORITY   11  /* 低优先级显示线程，确保控制任务绝对优先 */

static const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

/* 定义和静态初始化容量为 4 的内核消息队列 */
K_MSGQ_DEFINE(display_msg_q, sizeof(struct display_msg_packet), 4, 4);

void ssd1306_set_blanking(bool blank_on)
{
    if (device_is_ready(display_dev)) {
        if (blank_on) {
            display_blanking_on(display_dev);
        } else {
            display_blanking_off(display_dev);
        }
    }
}

/* 独立显示线程实现 */
void ssd1306_display_thread_entry(void *p1, void *p2, void *p3)
{
    struct display_msg_packet incoming_msg;

    /* 物理外设硬核初始化 */
    if (!device_is_ready(display_dev)) {
        printk("CRITICAL ERROR: SSD1306 Device tree node not ready!\n");
        return;
    }

    cfb_framebuffer_init(display_dev);
    cfb_set_kerning(display_dev, 0);

    /* 启动画面 */
    cfb_framebuffer_clear(display_dev, false);
    cfb_draw_text(display_dev, "SSD1306 THREAD", 0, 0);
    cfb_draw_text(display_dev, "INITIALIZED OK", 0, 18);
    cfb_framebuffer_finalize(display_dev);

    while (1) {
        /*
         * 从消息队列中死等屏显数据包。
         * 如果队列是空的，本线程交出 CPU 彻底进入内核休眠，零消耗功耗！
         */
        if (k_msgq_get(&display_msg_q, &incoming_msg, K_FOREVER) == 0) {

            /* 清空缓冲区 */
            cfb_framebuffer_clear(display_dev, false);

            /* 循环遍历 4 行文本，依次在不同的 Y 轴纵向坐标渲染 */
            for (int i = 0; i < DISPLAY_MAX_LINES; i++) {
                if (strlen(incoming_msg.lines[i]) > 0) {
                    /* 每行垂直间距跨度为 16 像素，完美对齐 8x16 字符 */
                    cfb_draw_text(display_dev, incoming_msg.lines[i], 0, i * 16);
                }
            }

            /* 一键刷入物理屏幕的电荷泵硬件寄存器 */
            cfb_framebuffer_finalize(display_dev);
        }
    }
}

/* 向 Zephyr 内核注册该专用显示线程 */
K_THREAD_DEFINE(ssd1306_display_tid, DISPLAY_THREAD_STACK_SIZE,
                ssd1306_display_thread_entry, NULL, NULL, NULL,
                DISPLAY_THREAD_PRIORITY, 0, 0);
