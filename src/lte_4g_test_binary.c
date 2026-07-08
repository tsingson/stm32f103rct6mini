#include <zephyr/kernel.h>
#include <string.h>
#include "lte_modem_interface.h"
#include "lte_gps_logic.h"
#include "lte_gps_report.h"
#include "gps_linq_n10.h"

#define TEST_SERVER_HOST "142.54.180.58"
#define TEST_SERVER_PORT 8061
#define DEFAULT_DEVICE_ID "xxxxxxx-5555"

// static void print_divider(const char* title)
// {
//     printk("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
//     printk("%s\n", title);
//     printk("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
// }

static void heartbeat_test(void)
{
    safe_log_publish("[4G-1] 心跳上报");

    heartbeat_report_t* hb = heartbeat_create(2, 78, 300, DEFAULT_DEVICE_ID);
    if (hb == NULL) {
        safe_log_publish("[4G-1] ❌ 创建失败\n");
        return;
    }

    safe_log_publish("[4G-1] 状态: %d (重要工作) | 电池: %d%% | 休眠: %d秒\n",
           hb->status, hb->battery, hb->sleep_countdown);

    if (heartbeat_send(hb, TEST_SERVER_HOST, TEST_SERVER_PORT)) {
        safe_log_publish("[4G-1] ✓ 成功发送\n");
    } else {
        safe_log_publish("[4G-1] ❌ 发送失败\n");
    }

    heartbeat_destroy(hb);
}

static void gps_test(void)
{
    safe_log_publish("[4G-2] GPS 定位上报");

    gps_report_t* report = gps_report_create(39.904030f, 116.407526f,
                                             get_system_timestamp(), 15, 45.3247f,
                                             DEFAULT_DEVICE_ID);
    if (report == NULL) {
        safe_log_publish("[4G-2] ❌ 创建失败\n");
        return;
    }

    gps_report_print(report);

    if (gps_report_send(report, TEST_SERVER_HOST, TEST_SERVER_PORT)) {
        safe_log_publish("[4G-2] ✓ 成功发送\n");
    } else {
        safe_log_publish("[4G-2] ❌ 发送失败\n");
    }

    gps_report_destroy(report);
}

static void ntp_test(void)
{
    safe_log_publish("[4G-0] NTP 时间同步");

    uint32_t ntp_time = 0;
    if (ntp_sync(&ntp_time)) {
        safe_log_publish("[4G-0] ✓ 收到时间戳: %u\n", ntp_time);
        safe_log_publish("[4G-0] ✓ 本地时间已同步\n");
    } else {
        safe_log_publish("[4G-0] ❌ 同步失败\n");
    }
}

static void lte_4g_binary_test_thread(void* p1, void* p2, void* p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    bool net_ready = false;

    k_sleep(K_MSEC(5000));
    safe_log_publish("\n[4G] 测试启动 (v3.0 - 模块化)\n");

    if (modem == NULL) {
        safe_log_publish("[4G] ❌ modem 驱动未初始化\n");
        return;
    }

    safe_log_publish("[4G] 驱动: %s\n", modem->name);

    while (1) {
        if (!net_ready) {
            safe_log_publish("[4G] 网络离线，重连中...\n");
            if (!modem->init_modem() || !modem->connect_network("ctnet")) {
                k_sleep(K_MSEC(3000));
                continue;
            }
            net_ready = true;
            safe_log_publish("[4G] ✓ 网络已连接\n");
        }

        if (!modem->check_alive()) {
            safe_log_publish("[4G] 网络掉线\n");
            net_ready = false;
            k_sleep(K_MSEC(2000));
            continue;
        }

        ntp_test();
        k_sleep(K_MSEC(2000));

        heartbeat_test();
        k_sleep(K_MSEC(3000));

        gps_test();
        k_sleep(K_MSEC(5000));
    }
}

// K_THREAD_DEFINE(lte_4g_test_tid, 4608, lte_4g_binary_test_thread, NULL, NULL, NULL, 4, 0, 0);
//
// void start_lte_4g_binary_test(void)
// {
//     k_thread_start(lte_4g_test_tid);
// }
