#ifndef GPS_DRIVER_H_
#define GPS_DRIVER_H_

#include <zephyr/kernel.h>

#define PRINT_STR_LEN       128
#define PRINT_QUEUE_DEPTH   64

typedef struct {
    char text[PRINT_STR_LEN];
} log_msg_t;

extern struct k_msgq print_msgq;
void safe_log_publish(const char *format, ...);

/* 🚀 新增：驱使编译器强制链接并保留该文件的显式初始化接口 */
void gps_driver_init(void);

#endif /* GPS_DRIVER_H_ */
