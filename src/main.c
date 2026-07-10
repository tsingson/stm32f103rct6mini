#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include "lis3dh.h"
#include "walk.h"
#include "ublox_m10_nano.h"
#include "gps_ring_buffer.h"
#include <zephyr/logging/log.h>
#include <stdlib.h>

#define SAMPLING_RATE_MS    (40)
#define LIS3DH_NODE         DT_NODELABEL(lis3dsh)

static const struct spi_dt_spec spi_dev = SPI_DT_SPEC_GET(LIS3DH_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
static const struct gpio_dt_spec int1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sensor_irq_gpios);

// ring buffer
GPS_RB_INSTANCE_DEFINE (gps_rb_main);
//
static ublox_m10_context_t gps_driver_ctx_main;
K_THREAD_STACK_DEFINE(gps_stack_main, GPS_THREAD_STACK_SZ);
//

// A. 从设备树安全获取两路不同的 STM32 硬件串口
const struct device* ublox_m10_gps_uart3_device = DEVICE_DT_GET(DT_NODELABEL(usart3));


int main(void)
{
    /**
     * gps
     */

    gps_rb_init(&gps_rb_main, gps_rb_main_raw_buf, sizeof(gps_rb_main_raw_buf));
    init_ubx_m10_driver_instance(&gps_driver_ctx_main, ublox_m10_gps_uart3_device, &gps_rb_main, gps_stack_main, 5);

    /**
    while(1) {
        gps_location_t main_loc;
        // 随时通过指定句柄消费对应实例的数据，两路完全并存、互不干扰
        if (gps_rb_pop(&gps_rb_main, &main_loc)) {
            // 消费主 GPS 数据...
        }
        k_sleep(K_MSEC(10));
    }
    */

    int16_t x_raw = 0, y_raw = 0, z_raw = 0;
    uint32_t total_steps = 0U, steps_inc = 0U;
    walk_pedometer_t my_pedometer;

    k_msleep(500);
    if (!spi_is_ready_dt(&spi_dev) || lis3dh_init(&spi_dev) < 0) return 0;
    lis3dh_reset_baseline(&spi_dev);
    gpio_pin_configure_dt(&int1_gpio, GPIO_INPUT | GPIO_PULL_DOWN);

    /* ================================================================= */
    /* 💡 全自动均值自适应初始化配置                                        */
    /* ================================================================= */
    uint32_t adaptive_debounce = 420U; /* 420ms 基础迈步防抖去回弹 */
    uint32_t adaptive_timeout = 4500U; /* 放宽断步超时时间为 4.5 秒，纵容慢速走停 */
    uint32_t min_amplitude = 140U; /* 幅度阻尼：动作起伏低于 140 LSB 的纯杂噪直接过滤 */

    /* 一键配置，算法内部将自动跟踪 Peak-Valley 并划定中间动态门槛 */
    walk_pedometer_init(&my_pedometer, 2, adaptive_debounce, adaptive_timeout, min_amplitude);
    /* ================================================================= */
    /* initial GPS u-blox m10 nano */
    printf("STM32F103_MINI 系统核心启动中...");


    while (1)
    {
        if (lis3dh_read_xyz(&spi_dev, &x_raw, &y_raw, &z_raw) == 0)
        {
            int64_t now_ms = k_uptime_get();
            (void)walk_pedometer_process(&my_pedometer, x_raw, y_raw, z_raw, now_ms, &steps_inc);

            if (steps_inc > 0U)
            {
                total_steps += steps_inc;
                if (steps_inc == (uint32_t)WALK_REQUIRED_STEPS)
                {
                    printf("\n🚀🚀🚀 [ADAPTIVE ACTIVE] 自动追踪步态成功！追加 %u 步。总步数: %u\n\n", WALK_REQUIRED_STEPS, total_steps);
                }
                else
                {
                    printf("🚶 [ADAPTIVE WALK] 规律波幅动态跨越，实时加步。总步数: %u\n", total_steps);
                }
            }
        }
        k_msleep(SAMPLING_RATE_MS);
    }
    return 0;
}
