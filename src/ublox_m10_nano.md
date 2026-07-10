```
#include "ublox_m10_nano.h"
#include "gps_ring_buffer.h"

// 1. 静态初始化 2 个完全隔离的多实例数据接收数据环 (256字节)
GPS_RB_INSTANCE_DEFINE(gps_rb_main);
GPS_RB_INSTANCE_DEFINE(gps_rb_backup);

// 2. 实例化 2 个完全独立的驱动软硬件控制上下文
static ublox_m10_context_t gps_driver_ctx_main;
static ublox_m10_context_t gps_driver_ctx_backup;

// 3. 为 2 个解包线程分别开辟相互独立的专用内核栈空间
K_THREAD_STACK_DEFINE(gps_stack_main, GPS_THREAD_STACK_SZ);
K_THREAD_STACK_DEFINE(gps_stack_backup, GPS_THREAD_STACK_SZ);

int main(void)
{
    // A. 从设备树安全获取两路不同的 STM32 硬件串口
    const struct device *uart2_device = DEVICE_DT_GET(DT_NODELABEL(uart2));
    const struct device *uart3_device = DEVICE_DT_GET(DT_NODELABEL(uart3));

    // B. 初始化多实例数据队列元数据
    gps_rb_init(&gps_rb_main, gps_rb_main_raw_buf, GPS_RING_BUF_BYTE_SIZE);
    gps_rb_init(&gps_rb_backup, gps_rb_backup_raw_buf, GPS_RING_BUF_BYTE_SIZE);

    // C. 强行将 [串口2] 解包绑定注入到 [主数据环]，并拉起独立高优解包线程1
    init_ubx_m10_driver_instance(&gps_driver_ctx_main, uart2_device, &gps_rb_main, gps_stack_main, 5);

    // D. 强行将 [串口3] 解包绑定注入到 [备用数据环]，并拉起独立高优解包线程2
    init_ubx_m10_driver_instance(&gps_driver_ctx_backup, uart3_device, &gps_rb_backup, gps_stack_backup, 5);

    while(1) {
        gps_location_t main_loc;
        // 随时通过指定句柄消费对应实例的数据，两路完全并存、互不干扰
        if (gps_rb_pop(&gps_rb_main, &main_loc)) {
            // 消费主 GPS 数据...
        }
        k_sleep(K_MSEC(10));
    }
    return 0;
}

```
