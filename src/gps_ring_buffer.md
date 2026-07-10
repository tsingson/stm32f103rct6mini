# gps ring buffer

```
#include "gps_ring_buffer.h"

// ⭐ 编译期静态定义两个完全独立隔离的 GPS 数据队列（main 队列与 backup 队列）
// 底层会自动在 BSS 段开辟 4 字节对齐的 256 字节物理内存，并自动平铺好元数据。
GPS_RB_INSTANCE_DEFINE(my_gps_main_rb);
GPS_RB_INSTANCE_DEFINE(my_gps_backup_rb);

void test_multi_instance(void)
{
    gps_location_t write_sample = { .fixType = 3, .numSV = 14, .lat = 31234567, .lon = 12145678, .gSpeed = 1200 };
    gps_location_t read_sample;

    // 1. 生产者直接向主队列实例压入数据
    (void)gps_rb_push(&my_gps_main_rb, &write_sample);

    // 2. 消费者从主队列实例弹出数据
    if (gps_rb_pop(&my_gps_main_rb, &read_sample) == true) {
         printk("成功读取主队列！当前主队列剩余积压: %u 帧\n", gps_rb_get_unread_count(&my_gps_main_rb));
    }
    
    // 3. 此时备用队列示例统计依然是 0，实现像素级完美解耦
    printk("备用队列剩余积压: %u 帧\n", gps_rb_get_unread_count(&my_gps_backup_rb));
}


```
