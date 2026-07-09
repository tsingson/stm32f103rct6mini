/* src/lis3dsh_app.c */
#include "lis3dsh_app.h"
#include "ssd1306_app.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LIS3DSH_WHO_AM_I        0x0F
#define LIS3DSH_CTRL_REG4       0x20
#define LIS3DSH_OUT_X_L         0x28
#define LIS3DSH_READ            BIT(7)
#define LIS3DSH_AUTO_INC        BIT(6)
#define LIS3DSH_WHO_AM_I_VALUE  0x3F

#define POLL_PERIOD_MS          100
#define MONITOR_WINDOW_MS       5000
#define REQUIRED_WALK_STEPS     4
#define MOVE_DELTA_THRESHOLD    900

static const struct device *const lis3dsh_spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec lis3dsh_cs = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
	.pin = 4,
	.dt_flags = GPIO_ACTIVE_LOW,
};

static struct spi_config lis3dsh_spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = 0,
	.cs = NULL,
};

bool is_elderly_moving = false;

static uint32_t stride_vibration_count;
static int16_t last_x, last_y, last_z;
static bool have_baseline;
static bool lis3dsh_ready;

static int lis3dsh_cs_assert(void)
{
	return gpio_pin_set_dt(&lis3dsh_cs, 0);
}

static int lis3dsh_cs_deassert(void)
{
	return gpio_pin_set_dt(&lis3dsh_cs, 1);
}

static int lis3dsh_reg_write(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = { reg & 0x3F, val };

	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = sizeof(tx),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};

	int ret = lis3dsh_cs_assert();
	if (ret < 0) {
		return ret;
	}

	ret = spi_write(lis3dsh_spi_dev, &lis3dsh_spi_cfg, &tx_set);
	(void)lis3dsh_cs_deassert();

	return ret;
}

static int lis3dsh_reg_read(uint8_t reg, uint8_t *val)
{
	uint8_t tx[2] = { reg | LIS3DSH_READ, 0x00 };
	uint8_t rx[2] = { 0 };

	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = sizeof(tx),
	};
	const struct spi_buf rx_buf = {
		.buf = rx,
		.len = sizeof(rx),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	int ret = lis3dsh_cs_assert();
	if (ret < 0) {
		return ret;
	}

	ret = spi_transceive(lis3dsh_spi_dev, &lis3dsh_spi_cfg, &tx_set, &rx_set);
	(void)lis3dsh_cs_deassert();

	if (ret == 0) {
		*val = rx[1];
	}

	return ret;
}

static int lis3dsh_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tx[7] = {
		LIS3DSH_OUT_X_L | LIS3DSH_READ | LIS3DSH_AUTO_INC,
		0, 0, 0, 0, 0, 0
	};
	uint8_t rx[7] = { 0 };

	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = sizeof(tx),
	};
	const struct spi_buf rx_buf = {
		.buf = rx,
		.len = sizeof(rx),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	int ret = lis3dsh_cs_assert();
	if (ret < 0) {
		return ret;
	}

	ret = spi_transceive(lis3dsh_spi_dev, &lis3dsh_spi_cfg, &tx_set, &rx_set);
	(void)lis3dsh_cs_deassert();

	if (ret < 0) {
		return ret;
	}

	*x = (int16_t)((rx[2] << 8) | rx[1]);
	*y = (int16_t)((rx[4] << 8) | rx[3]);
	*z = (int16_t)((rx[6] << 8) | rx[5]);

	return 0;
}

static void lis3dsh_publish_state(bool moving, uint32_t steps)
{
	struct display_msg_packet msg;

	memset(&msg, 0, sizeof(msg));
	strncpy(msg.lines[0], "RADAR MONITOR", DISPLAY_LINE_MAX_LEN);

	if (moving) {
		strncpy(msg.lines[1], "STATE: MOVING!", DISPLAY_LINE_MAX_LEN);
		snprintf(msg.lines[2], DISPLAY_LINE_MAX_LEN, "STEPS ACC: %u", steps);
		strncpy(msg.lines[3], "4G/GPS: READY", DISPLAY_LINE_MAX_LEN);
	} else {
		strncpy(msg.lines[1], "STATE: NO MOVE", DISPLAY_LINE_MAX_LEN);
		strncpy(msg.lines[2], "STABLE WINDOW", DISPLAY_LINE_MAX_LEN);
		strncpy(msg.lines[3], "4G/GPS: SILENT", DISPLAY_LINE_MAX_LEN);
	}

	k_msgq_put(&display_msg_q, &msg, K_NO_WAIT);
}

static void lis3dsh_poll_thread_entry(void *p1, void *p2, void *p3)
{
	int64_t window_start = k_uptime_get();

	while (1) {
		if (!lis3dsh_ready) {
			k_msleep(100);
			continue;
		}

		int16_t x, y, z;
		if (lis3dsh_read_xyz(&x, &y, &z) == 0) {
			if (!have_baseline) {
				last_x = x;
				last_y = y;
				last_z = z;
				have_baseline = true;
			} else {
				int32_t delta = abs(x - last_x) + abs(y - last_y) + abs(z - last_z);
				if (delta > MOVE_DELTA_THRESHOLD) {
					stride_vibration_count++;
				}
				last_x = x;
				last_y = y;
				last_z = z;
			}
		}

		int64_t now = k_uptime_get();
		if ((now - window_start) >= MONITOR_WINDOW_MS) {
			is_elderly_moving = (stride_vibration_count >= REQUIRED_WALK_STEPS);
			lis3dsh_publish_state(is_elderly_moving, stride_vibration_count);

			stride_vibration_count = 0;
			window_start = now;
		}

		k_msleep(POLL_PERIOD_MS);
	}
}

K_THREAD_DEFINE(lis3dsh_poll_tid, 1024, lis3dsh_poll_thread_entry,
		NULL, NULL, NULL, 9, 0, 0);

int lis3dsh_motion_init(void)
{
	if (!device_is_ready(lis3dsh_spi_dev)) {
		printk("LIS3DSH SPI bus not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(lis3dsh_cs.port)) {
		printk("LIS3DSH CS GPIO not ready\n");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&lis3dsh_cs, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("LIS3DSH CS GPIO configure failed: %d\n", ret);
		return ret;
	}

	uint8_t who_am_i = 0;
	ret = lis3dsh_reg_read(LIS3DSH_WHO_AM_I, &who_am_i);
	if (ret < 0) {
		printk("LIS3DSH WHO_AM_I read failed: %d\n", ret);
		return ret;
	}

	if (who_am_i != LIS3DSH_WHO_AM_I_VALUE) {
		printk("LIS3DSH WHO_AM_I mismatch: 0x%02x\n", who_am_i);
		return -EIO;
	}

	ret = lis3dsh_reg_write(LIS3DSH_CTRL_REG4, 0x67);
	if (ret < 0) {
		printk("LIS3DSH CTRL_REG4 write failed: %d\n", ret);
		return ret;
	}

	lis3dsh_ready = true;
	printk("LIS3DSH SPI init OK\n");
	return 0;
}
