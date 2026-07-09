/* src/ssd1306_app.h */
#ifndef SSD1306_APP_H_
#define SSD1306_APP_H_

#include <zephyr/kernel.h>

#define DISPLAY_LINE_MAX_LEN 16
#define DISPLAY_MAX_LINES    4

/* 传递给显示线程的屏显数据包 */
struct display_msg_packet {
    char lines[DISPLAY_MAX_LINES][DISPLAY_LINE_MAX_LEN + 1];
};

/* 声明一个容量为 4 的显示内核消息队列，供外部线程投递文本 */
extern struct k_msgq display_msg_q;

/* 导出控制屏幕物理开/关的接口，供低功耗休眠拦截调用 */
void ssd1306_set_blanking(bool blank_on);

#endif /* SSD1306_APP_H_ */
