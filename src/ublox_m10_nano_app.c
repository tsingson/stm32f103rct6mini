/**
 * @file main.c
 * @brief Zephyr 4.4.1 GPS 消费者应用线程业务入口 (符合 C17)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include "ublox_m10_nano.h"
#include "gps_ring_buffer.h"

LOG_MODULE_REGISTER(gps_app, CONFIG_GPS_LOG_LEVEL);

#define CONSUMER_STACK_SIZE 2048U
#define CONSUMER_PRIORITY   6

// ==============================================================================
// 1. 声明线程函数（必须在使用宏之前被编译器看到原型）
// ==============================================================================
void gps_consumer_thread_entry(void* p1, void* p2, void* p3);
// ==============================================================================
// 1. 去掉 static 关键字，严格对齐 Zephyr 的 k_thread_entry_t 标准函数签名
// ==============================================================================
void gps_consumer_thread_entry(void* p1, void* p2, void* p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    gps_location_t received_data;

    LOG_WRN("[CONSUMER] 正在进行鲁棒性抗过载压力测试：强制挂起 2 秒...");
    k_sleep(K_MSEC(2000));
    LOG_INF("[CONSUMER] 测试挂起结束！开始全力清空历史积压数据...");

    while (1)
    {
        if (gps_rb_pop(&received_data) == true)
        {
            int32_t lat_val = received_data.lat;
            int32_t lon_val = received_data.lon;
            int32_t speed_kh_x100 = (int32_t)(((int64_t)received_data.gSpeed * 360LL) / 1000LL);

            printk("[UBX 5Hz 消费者消费] 卫星: %u | 定位类型: %u | "
                   "纬度: %d.%07ld | 经度: %d.%07ld | 地速: %d.%02ld km/h | 队列剩余: %u\n",
                   (unsigned int)received_data.numSV,
                   (unsigned int)received_data.fixType,
                   lat_val / 10000000, labs(lat_val % 10000000),
                   lon_val / 10000000, labs(lon_val % 10000000),
                   speed_kh_x100 / 100, labs(speed_kh_x100 % 100),
                   (unsigned int)gps_rb_get_unread_count());

            k_sleep(K_MSEC(1));
        }
        else
        {
            LOG_INF("[CONSUMER] 缓冲区积压全部清除完毕。恢复常规松散监听中...");
            k_sleep(K_MSEC(200));
        }
    }
}


K_THREAD_DEFINE(gps_consumer_thread_id, 1024, gps_consumer_thread_entry,
                NULL, NULL, NULL, 9, 0, 0);


// ==============================================================================
// 系统主入口
// ==============================================================================
/**
int main(void)
{
    LOG_INF("STM32F103_MINI 系统核心启动中...");

    // 1. 初始化 Zephyr 原生对象环形缓冲区
    gps_rb_init();

    // 2. 启动 u-blox 串口中断底层接收内核 (内部会自动创建专属解包线程，无需您手动写 task 轮询)
    int ret = init_ubx_nona_gps_uart();
    if (ret != 0) {
        LOG_ERR("GPS 串口中断内核拉起失败: %d", ret);
        return ret;
    }

    LOG_INF("驱动解包内核已成功挂载，应用主线程进入低功耗休眠.");
    return 0;
}
*/
