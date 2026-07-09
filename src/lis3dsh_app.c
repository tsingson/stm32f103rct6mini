/* src/lis3dsh_app.c */
#include "lis3dsh_app.h"
#include "ssd1306_app.h" /* 引入我们上一轮拆分出来的显示队列抽象 */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>


// static const struct device *lis3dsh_dev = DEVICE_DT_GET(DT_NODELABEL(lis3dsh));
static const struct device *lis3dsh_dev;

bool is_elderly_moving = false;

/* 内部状态运动积分监控变量 */
static uint32_t stride_vibration_count = 0;
static int64_t last_isr_timestamp = 0;

#define MONITOR_WINDOW_MS   5000  /* 5秒滑动时间校验窗（约对应行走 5 米的空间跨度） */
#define REQUIRED_WALK_STEPS 4     /* 5秒内必须至少连续踏出 4 步冲击，才确认为真实位移 */

/*
 * LIS3DSH 硬件硬中断服务程序 (ISR Handler)
 * 当老人上下楼梯产生重力颠簸、或左右步行晃动时，LIS3DSH 内部硬件状态机
 * 瞬间拉高 INT1 引脚，STM32 在硬件层无阻塞进入本函数。
 */
static void lis3dsh_trigger_handler(const struct device *dev,
                                    const struct sensor_trigger *trig)
{
    int64_t current_time = k_uptime_get();
    struct display_msg_packet msg;

    /* 20ms 内的极速消抖，防止金属弹片或杂散电气噪声引发二次虚假中断 */
    if ((current_time - last_isr_timestamp) < 20) {
        return;
    }
    last_isr_timestamp = current_time;

    /* 步伐冲击积分累加 */
    stride_vibration_count++;

    /* 核心逻辑核查：5秒滑动窗口内，连续跨步数突破 4 步 */
    if (stride_vibration_count >= REQUIRED_WALK_STEPS) {
        is_elderly_moving = true;

        /* 异步封装报文，直接投递给低优先级的屏幕线程刷新显示 */
        memset(&msg, 0, sizeof(msg));
        strncpy(msg.lines[0], "RADAR MONITOR", DISPLAY_LINE_MAX_LEN);
        strncpy(msg.lines[1], "STATE: MOVING!", DISPLAY_LINE_MAX_LEN);
        snprintf(msg.lines[2], DISPLAY_LINE_MAX_LEN, "STEPS ACC: %d", (int)stride_vibration_count);
        strncpy(msg.lines[3], "4G/GPS: READY", DISPLAY_LINE_MAX_LEN);

        k_msgq_put(&display_msg_q, &msg, K_NO_WAIT);
    }
}

/* 监控常驻工作线程：专门负责在 5秒 滑动窗口到期时，检测并强制归位为 no move */
void lis3dsh_monitor_thread_entry(void *p1, void *p2, void *p3)
{
    struct display_msg_packet msg;

    while (1) {
        /* 每隔 5 秒钟进行一次周期性考核 */
        k_msleep(MONITOR_WINDOW_MS);

        if (stride_vibration_count < REQUIRED_WALK_STEPS) {
            /*
             * 如果 5 秒内步伐冲击次数不足 4 次，说明老人处于原地静止、安稳坐姿，
             * 或者只是 5 米范围内的微小动作、微幅翻身，一律判定为 no move，拦截耗电外设。
             */
            is_elderly_moving = false;

            memset(&msg, 0, sizeof(msg));
            strncpy(msg.lines[0], "RADAR MONITOR", DISPLAY_LINE_MAX_LEN);
            strncpy(msg.lines[1], "STATE: NO MOVE", DISPLAY_LINE_MAX_LEN);
            strncpy(msg.lines[2], "STABLE WINDOW", DISPLAY_LINE_MAX_LEN);
            strncpy(msg.lines[3], "4G/GPS: SILENT", DISPLAY_LINE_MAX_LEN);

            k_msgq_put(&display_msg_q, &msg, K_NO_WAIT);
        }

        /* 时窗结束，强制积分清零，无条件进入下一个 5 秒的时间轴周期考核 */
        stride_vibration_count = 0;
    }
}

int lis3dsh_motion_init(void)
{
    // 新增：动态绑定设备
    lis3dsh_dev = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(lis3dsh)));
    if (lis3dsh_dev == NULL) {
        printk("CRITICAL ERROR: LIS3DSH driver instance not found in Zephyr 4.4.1!\n");
        return -ENODEV;
    }

    if (!device_is_ready(lis3dsh_dev)) {
        printk("CRITICAL ERROR: LIS3DSH I2C Device hardware not ready!\n");
        return -ENODEV; // 将原来的 -1 改为 -ENODEV
    }
    // ... 后续代码


    /* 配置 LIS3DSH 的硬件触发器属性：设置为任何轴发生数据跳变（DELTA）时触发 */
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DELTA,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    /* 将我们的硬件 ISR 回调函数正式注册挂载到内核的传感器抽象管理链表中 */
    int ret = sensor_trigger_set(lis3dsh_dev, &trig, lis3dsh_trigger_handler);
    if (ret < 0) {
        printk("Error: Failed to bind LIS3DSH interrupt trigger (err %d)\n", ret);
        return ret;
    }

    printk("LIS3DSH Any-Motion Hardware Ext-Interrupt configured and armed.\n");
    return 0;
}

/* 注册低功耗判定监控滑动窗口线程 */
K_THREAD_DEFINE(lis3dsh_monitor_tid, 1024, lis3dsh_monitor_thread_entry,
                NULL, NULL, NULL, 9, 0, 0); /* 优先级 9，确保判定时效性 */
