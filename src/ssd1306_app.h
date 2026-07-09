/* src/ssd1306_app.h */
#ifndef SSD1306_APP_H_
#define SSD1306_APP_H_

#include <zephyr/kernel.h>

#define DISPLAY_LINE_MAX_LEN 16
#define DISPLAY_MAX_LINES    6  /* 核心升级：从小字体开始，支持同屏显示最多 6 行密集物联网文本 */

struct display_msg_packet {
    char lines[DISPLAY_MAX_LINES][DISPLAY_LINE_MAX_LEN + 1];
};

extern struct k_msgq display_msg_q;

void ssd1306_set_blanking(bool blank_on);

#endif /* SSD1306_APP_H_ */
