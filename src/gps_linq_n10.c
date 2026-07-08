#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <zephyr/sys/atomic.h>
#include "gps_linq_n10.h"
#include "gps_nmea.h"

#define UART_DEVICE_NODE       DT_ALIAS(gps_uart)
#define N10_W_NODE             DT_NODELABEL(n10_w_pin)
#define NMEA_BUF_SIZE          128

#define TARGET_GPS_COUNT       30
#define FIRST_FIX_TIMEOUT_MS   90000
#define RUNTIME_RETRY_LIMIT    3
#define RUNTIME_FIX_TIMEOUT_MS 10000
#define SLEEP_INTERVAL_MS      180000

K_MSGQ_DEFINE(raw_uart_msgq, NMEA_BUF_SIZE, 8, 4);
K_MSGQ_DEFINE(print_msgq, sizeof(log_msg_t), PRINT_QUEUE_DEPTH, 4);

static const struct gpio_dt_spec n10_w_gpio = GPIO_DT_SPEC_GET(N10_W_NODE, gpios);
static char rx_buffer[NMEA_BUF_SIZE];
static int rx_buf_idx = 0;
static bool rx_drop_until_eol = false;
static atomic_t uart_rx_overflow_count;
static atomic_t uart_rx_line_drop_count;
static atomic_t log_queue_drop_count;



static void gps_pwr_on(void) {
    (void)gpio_pin_set_dt(&n10_w_gpio, 1);
}


static void gps_pwr_off  (void) {
    (void)gpio_pin_set_dt(&n10_w_gpio, 0);
}




static bool should_print_speed_detail(double speed_kmh)
{
    return IS_ENABLED(CONFIG_GPS_SPEED_FULL_OUTPUT) ||
        (speed_kmh >= (double)CONFIG_GPS_SPEED_LOG_THRESHOLD_KMH);
}

static void n10_send_pmtk(const struct device* uart_dev, const char* cmd)
{
    char tx_buf[64];
    uint8_t checksum = 0;
    size_t tx_len;

    for (size_t i = 0; cmd[i] != '\0'; i++) { checksum ^= (uint8_t)cmd[i]; }
    snprintf(tx_buf, sizeof(tx_buf), "$%s*%02X\r\n", cmd, checksum);
    tx_len = strnlen(tx_buf, sizeof(tx_buf));
    for (size_t i = 0; i < tx_len; i++) { uart_poll_out(uart_dev, tx_buf[i]); }
    k_sleep(K_MSEC(200));
}

void safe_log_publish(const char* format, ...)
{
    log_msg_t msg;
    va_list args;
    va_start(args, format);
    vsnprintf(msg.text, sizeof(msg.text), format, args);
    va_end(args);

    if (k_msgq_put(&print_msgq, &msg, K_NO_WAIT) != 0)
    {
        log_msg_t dummy;

        atomic_inc(&log_queue_drop_count);
        if (k_msgq_get(&print_msgq, &dummy, K_NO_WAIT) == 0)
        {
            if (k_msgq_put(&print_msgq, &msg, K_NO_WAIT) != 0)
            {
                atomic_inc(&log_queue_drop_count);
            }
        }
    }
}

static void uart_cb(const struct device* dev, void* user_data)
{
    uint8_t c;
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) return;

    while (uart_fifo_read(dev, &c, 1) > 0)
    {
        if (rx_drop_until_eol)
        {
            if (c == '\n' || c == '\r')
            {
                rx_drop_until_eol = false;
                rx_buf_idx = 0;
            }
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            if (rx_buf_idx > 0)
            {
                int rc;
                rx_buffer[rx_buf_idx] = '\0';
                rc = k_msgq_put(&raw_uart_msgq, &rx_buffer, K_NO_WAIT);
                if (rc == -ENOMSG)
                {
                    atomic_inc(&uart_rx_line_drop_count);
                }
                rx_buf_idx = 0;
            }
        }
        else if (rx_buf_idx < (NMEA_BUF_SIZE - 1))
        {
            rx_buffer[rx_buf_idx++] = c;
        }
        else
        {
            atomic_inc(&uart_rx_overflow_count);
            rx_drop_until_eol = true;
            rx_buf_idx = 0;
        }
    }
}

static void gps_control_thread(void* p1, void* p2, void* p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    const struct device* const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
    char nmea_sentence[NMEA_BUF_SIZE];

    if (!gpio_is_ready_dt(&n10_w_gpio))
    {
        printk("[4G-L][ERR] n10_w_gpio not ready.\n");
        return;
    }
    (void)gpio_pin_configure_dt(&n10_w_gpio, GPIO_OUTPUT_ACTIVE);

    if (!device_is_ready(uart_dev))
    {
        printk("[4G-L][ERR] gps uart not ready: %s\n", uart_dev->name);
        return;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    safe_log_publish("[4G-L] Init done.");

    while (1)
    {
        safe_log_publish("[PWR] ON.");
        (void)gps_pwr_on();
        k_sleep(K_MSEC(1500));

        n10_send_pmtk(uart_dev, "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
        k_msgq_purge(&raw_uart_msgq);

        int current_sats = 0;
        char current_time[10] = "000000";
        double current_lat = 0.0, current_lon = 0.0;
        double current_speed_knots = 0.0;
        bool has_first_fix = false;
        int64_t upper_start_time = k_uptime_get();
        int64_t last_heartbeat_log_ms = 0;
        int overflow_snapshot = atomic_get(&uart_rx_overflow_count);
        int line_drop_snapshot = atomic_get(&uart_rx_line_drop_count);
        int log_drop_snapshot = atomic_get(&log_queue_drop_count);
        int first_fix_elapsed_ms = -1;
        int captured_frames = 0;
        char obs_summary[256];
        nmea_parse_state_t parse_state;
        nmea_parse_config_t parse_cfg;
        nmea_obs_stats_t obs;

        nmea_reset_parse_state(&parse_state);
        nmea_get_default_config(&parse_cfg);
        nmea_reset_obs_stats(&obs);

        double current_speed_kmh = nmea_knots_to_kmh(current_speed_knots);
        double current_speed_ms = nmea_knots_to_ms(current_speed_knots);

        safe_log_publish("[4G-L] Wait satnav (%ds)...", FIRST_FIX_TIMEOUT_MS / 1000);

        while (k_uptime_get() - upper_start_time < FIRST_FIX_TIMEOUT_MS)
        {
            int32_t rem_time = FIRST_FIX_TIMEOUT_MS - (int32_t)(k_uptime_get() - upper_start_time);
            if (rem_time <= 0) break;

            if (k_msgq_get(&raw_uart_msgq, &nmea_sentence, K_MSEC(1000)) == 0)
            {
                int64_t now_ms = k_uptime_get();
                int current_overflow = atomic_get(&uart_rx_overflow_count);

                if ((now_ms - last_heartbeat_log_ms) >= 8000)
                {
                    last_heartbeat_log_ms = now_ms;
                    safe_log_publish("[UART] Alive, waiting fix...");
                }
                if (current_overflow != overflow_snapshot)
                {
                    safe_log_publish("[WARN] UART overflow: %d", current_overflow);
                    overflow_snapshot = current_overflow;
                }

                if (nmea_process_sentence(nmea_sentence, &parse_state, &parse_cfg, &obs, &current_sats,
                                          current_time, sizeof(current_time),
                                          &current_speed_knots,
                                          &current_lat, &current_lon))
                {
                    current_speed_kmh = nmea_knots_to_kmh(current_speed_knots);
                    current_speed_ms = nmea_knots_to_ms(current_speed_knots);
                    has_first_fix = true;
                    first_fix_elapsed_ms = (int)(k_uptime_get() - upper_start_time);
                    if (should_print_speed_detail(current_speed_kmh))
                    {
                        safe_log_publish("[FIX] Star=%0d | Speed=%.2f kn (%.2f km/h / %.2f m/s)",
                                         current_sats, current_speed_knots, current_speed_kmh, current_speed_ms);
                    }
                    else
                    {
                        safe_log_publish("[FIX] Star=%0d | Speed=%.2f km/h", current_sats, current_speed_kmh);
                    }
                    break;
                }
            }
            else
            {
                obs.msgq_timeout_total++;
            }
        }

        bool session_success = false;
        if (has_first_fix)
        {
            int successful_gps_frames = 0;
            int runtime_retry_count = 0;
            safe_log_publish("[GPS] Capture %d fixes...", TARGET_GPS_COUNT);
            while (runtime_retry_count < RUNTIME_RETRY_LIMIT)
            {
                int64_t frame_start_time = k_uptime_get();
                bool got_frame = false;
                while (k_uptime_get() - frame_start_time < RUNTIME_FIX_TIMEOUT_MS)
                {
                    int32_t rem_time = RUNTIME_FIX_TIMEOUT_MS - (int32_t)(k_uptime_get() - frame_start_time);
                    if (rem_time <= 0) break;
                    if (k_msgq_get(&raw_uart_msgq, &nmea_sentence, K_MSEC(rem_time)) == 0)
                    {
                        if (nmea_process_sentence(nmea_sentence, &parse_state, &parse_cfg, &obs, &current_sats,
                                                  current_time, sizeof(current_time),
                                                  &current_speed_knots,
                                                  &current_lat, &current_lon))
                        {
                            current_speed_kmh = nmea_knots_to_kmh(current_speed_knots);
                            current_speed_ms = nmea_knots_to_ms(current_speed_knots);
                            got_frame = true;
                            successful_gps_frames++;
                            captured_frames = successful_gps_frames;
                            if (successful_gps_frames == 1 ||
                                successful_gps_frames == TARGET_GPS_COUNT ||
                                (successful_gps_frames % 5) == 0)
                            {
                                if (should_print_speed_detail(current_speed_kmh))
                                {
                                    safe_log_publish(
                                        "[DATA] %d/%d Star=%0d Speed=%.2f kn/%.2f km/h/%.2f m/s Lon=%.6f Lat=%.6f",
                                        successful_gps_frames, TARGET_GPS_COUNT,
                                        current_sats,
                                        current_speed_knots,
                                        current_speed_kmh,
                                        current_speed_ms,
                                        current_lon, current_lat);
                                }
                                else
                                {
                                    safe_log_publish(
                                        "[DATA] %d/%d Star=%0d Speed=%.2f km/h Lon=%.6f Lat=%.6f",
                                        successful_gps_frames, TARGET_GPS_COUNT,
                                        current_sats,
                                        current_speed_kmh,
                                        current_lon, current_lat);
                                }
                            }
                            if (successful_gps_frames >= TARGET_GPS_COUNT)
                            {
                                session_success = true;
                                break;
                            }
                            frame_start_time = k_uptime_get();
                        }
                    }
                    else
                    {
                        obs.msgq_timeout_total++;
                    }
                }
                if (session_success) break;
                if (!got_frame)
                {
                    runtime_retry_count++;
                    obs.signal_lost_total++;
                    safe_log_publish("[WARN] Signal lost %d/%d.", runtime_retry_count, RUNTIME_RETRY_LIMIT);
                }
            }
        }
        else { safe_log_publish("[FAIL] No fix in %ds.", FIRST_FIX_TIMEOUT_MS / 1000); }

        if (session_success) { safe_log_publish("[SESSION] OK. Power off."); }
        else { safe_log_publish("[SESSION] FAIL. Power off."); }

        safe_log_publish("[PWR] OFF.");
        (void)gps_pwr_off( );

        nmea_build_observability_summary(
            &obs,
            (int)(k_uptime_get() - upper_start_time),
            first_fix_elapsed_ms,
            captured_frames,
            atomic_get(&uart_rx_overflow_count) - overflow_snapshot,
            atomic_get(&uart_rx_line_drop_count) - line_drop_snapshot,
            atomic_get(&log_queue_drop_count) - log_drop_snapshot,
            session_success,
            obs_summary,
            sizeof(obs_summary));
        safe_log_publish("%s", obs_summary);

        // safe_log_publish("[PWR] Sleep 3 min.");
        // k_sleep(K_MSEC(SLEEP_INTERVAL_MS));
    }
}


void gps_driver_init(void)
{
    safe_log_publish("[BOOT] GPS driver ready.\n");
}

K_THREAD_DEFINE(gps_manager_id, 2048, gps_control_thread, NULL, NULL, NULL, 5, 0, 0);
