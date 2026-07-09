/* src/ssd1306_app.c */
#include "ssd1306_app.h"
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <stdio.h>
#include <string.h>

#define DISPLAY_THREAD_STACK_SIZE 1024
#define DISPLAY_THREAD_PRIORITY   11

static const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

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

void ssd1306_display_thread_entry(void *p1, void *p2, void *p3)
{
    struct display_msg_packet incoming_msg;
    uint8_t selected_font_idx = 0;
    uint8_t min_font_height = 255; /* 动态锚定最小字体高度 */

    if (!device_is_ready(display_dev)) {
        return;
    }

    cfb_framebuffer_init(display_dev);
    cfb_set_kerning(display_dev, 0);

    /*
     * ==================== Zephyr 4.4.1 动态自适应字号寻找 ====================
     * 盘点全板加载的所有字库，找出高度(height)最小的那款字库，将其索引记录下来。
     */
    uint8_t num_fonts = cfb_get_numof_fonts(display_dev);
    for (uint8_t i = 0; i < num_fonts; i++) {
        uint8_t width, height;
        cfb_get_font_size(display_dev, i, &width, &height);

        if (height < min_font_height) {
            min_font_height = height;
            selected_font_idx = i;
        }
    }

    /* 显式应用这款全板最小的 8 像素高字体 */
    cfb_framebuffer_set_font(display_dev, selected_font_idx);

    cfb_framebuffer_clear(display_dev, false);
    cfb_draw_text(display_dev, "OLED 8X8 SYSTEM", 0, 0);
    cfb_framebuffer_finalize(display_dev);

    while (1) {
        if (k_msgq_get(&display_msg_q, &incoming_msg, K_FOREVER) == 0) {

            cfb_framebuffer_clear(display_dev, false);

            /*
             * 动态排版：行间距跟随刚才盘点出来的实际最小字体高度 `min_font_height` 走！
             * 让这块 128x64 的小屏幕展现出最高的信息密集度。
             */
            for (int i = 0; i < DISPLAY_MAX_LINES; i++) {
                if (strlen(incoming_msg.lines[i]) > 0) {
                    cfb_draw_text(display_dev, incoming_msg.lines[i], 0, i * (min_font_height + 2));
                }
            }

            cfb_framebuffer_finalize(display_dev);
        }
    }
}

K_THREAD_DEFINE(ssd1306_display_tid, DISPLAY_THREAD_STACK_SIZE,
                ssd1306_display_thread_entry, NULL, NULL, NULL,
                DISPLAY_THREAD_PRIORITY, 0, 0);
