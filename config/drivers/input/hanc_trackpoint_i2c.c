/*
 * Hanc custom I2C trackpoint driver.
 *
 * Polls an ATtiny co-processor that exposes a 10-byte report:
 *   [0]     seq
 *   [1..2]  raw dx (int16 LE)
 *   [3..4]  raw dy (int16 LE)
 *   [5..6]  raw x  (int16 LE, unused here)
 *   [7..8]  raw y  (int16 LE, unused here)
 *   [9]     buttons bitmap (bit0=L, bit1=R, bit2=M)
 *
 * Matches the dynamic-poll + Q8 baseline-drift scheme from the Arduino
 * reference implementation.
 */

#define DT_DRV_COMPAT hanc_trackpoint_i2c

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(hanc_trackpoint_i2c, CONFIG_HANC_TRACKPOINT_I2C_LOG_LEVEL);

#define OFFSET_FILTER_SHIFT 4
#define REPORT_LEN 10
#define I2C_ERROR_BACKOFF_MS 250

struct trackpoint_config {
	struct i2c_dt_spec bus;
	uint32_t poll_idle_ms;
	uint32_t poll_active_ms;
	uint32_t idle_timeout_ms;
	int32_t dead_zone;
	int32_t max_speed;
	bool invert_x;
	bool invert_y;
};

struct trackpoint_data {
	const struct device *dev;
	struct k_work_delayable work;
	uint32_t current_poll_ms;
	int64_t last_move_time;
	int32_t dx_offset_q8;
	int32_t dy_offset_q8;
	uint8_t last_buttons;
	bool baseline_seeded;
};

static inline int16_t clamp_speed(int32_t v, int32_t limit)
{
	if (v > limit) {
		return (int16_t)limit;
	}
	if (v < -limit) {
		return (int16_t)(-limit);
	}
	return (int16_t)v;
}

static void trackpoint_emit(const struct device *dev, int16_t dx, int16_t dy,
			    uint8_t buttons, uint8_t last_buttons)
{
	uint8_t diff = (buttons ^ last_buttons) & 0x07;
	int remaining = (dx ? 1 : 0) + (dy ? 1 : 0) + POPCOUNT(diff);

	if (remaining == 0) {
		return;
	}

	if (dx) {
		remaining--;
		input_report_rel(dev, INPUT_REL_X, dx, remaining == 0, K_NO_WAIT);
	}
	if (dy) {
		remaining--;
		input_report_rel(dev, INPUT_REL_Y, dy, remaining == 0, K_NO_WAIT);
	}
	if (diff & 0x01) {
		remaining--;
		input_report_key(dev, INPUT_BTN_0, (buttons & 0x01) ? 1 : 0,
				 remaining == 0, K_NO_WAIT);
	}
	if (diff & 0x02) {
		remaining--;
		input_report_key(dev, INPUT_BTN_1, (buttons & 0x02) ? 1 : 0,
				 remaining == 0, K_NO_WAIT);
	}
	if (diff & 0x04) {
		remaining--;
		input_report_key(dev, INPUT_BTN_2, (buttons & 0x04) ? 1 : 0,
				 remaining == 0, K_NO_WAIT);
	}
}

static void trackpoint_poll(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct trackpoint_data *data =
		CONTAINER_OF(dwork, struct trackpoint_data, work);
	const struct device *dev = data->dev;
	const struct trackpoint_config *cfg = dev->config;

	uint8_t buf[REPORT_LEN];
	int ret = i2c_read_dt(&cfg->bus, buf, sizeof(buf));
	if (ret < 0) {
		LOG_WRN("i2c read failed: %d", ret);
		k_work_reschedule(dwork, K_MSEC(I2C_ERROR_BACKOFF_MS));
		return;
	}

	int16_t raw_dx = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
	int16_t raw_dy = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
	uint8_t buttons = buf[9];

	/*
	 * Seed the baseline from the first sample so we don't flood the host
	 * with motion while the Q8 filter (which only tracks within the dead
	 * zone) is converging from zero.
	 */
	if (!data->baseline_seeded) {
		data->dx_offset_q8 = ((int32_t)raw_dx) << 8;
		data->dy_offset_q8 = ((int32_t)raw_dy) << 8;
		data->baseline_seeded = true;
	}

	int16_t offset_dx = (int16_t)((data->dx_offset_q8 + 128) >> 8);
	int16_t offset_dy = (int16_t)((data->dy_offset_q8 + 128) >> 8);
	int16_t comp_dx = raw_dx - offset_dx;
	int16_t comp_dy = raw_dy - offset_dy;

	/*
	 * Learn drift whenever we're sitting in the dead zone, regardless of
	 * polling mode -- otherwise baseline warming during active use never
	 * gets re-tracked.
	 */
	if (buttons == 0 &&
	    abs(comp_dx) < cfg->dead_zone && abs(comp_dy) < cfg->dead_zone) {
		int32_t target_dx_q8 = ((int32_t)raw_dx) << 8;
		int32_t target_dy_q8 = ((int32_t)raw_dy) << 8;
		data->dx_offset_q8 +=
			(target_dx_q8 - data->dx_offset_q8) >> OFFSET_FILTER_SHIFT;
		data->dy_offset_q8 +=
			(target_dy_q8 - data->dy_offset_q8) >> OFFSET_FILTER_SHIFT;

		offset_dx = (int16_t)((data->dx_offset_q8 + 128) >> 8);
		offset_dy = (int16_t)((data->dy_offset_q8 + 128) >> 8);
		comp_dx = raw_dx - offset_dx;
		comp_dy = raw_dy - offset_dy;
	}

	int16_t dx = 0;
	int16_t dy = 0;
	if (abs(comp_dx) >= cfg->dead_zone || abs(comp_dy) >= cfg->dead_zone) {
		int32_t sx = cfg->invert_x ? -(int32_t)comp_dx : (int32_t)comp_dx;
		int32_t sy = cfg->invert_y ? -(int32_t)comp_dy : (int32_t)comp_dy;
		dx = clamp_speed(sx, cfg->max_speed);
		dy = clamp_speed(sy, cfg->max_speed);
	}

	trackpoint_emit(dev, dx, dy, buttons, data->last_buttons);
	data->last_buttons = buttons;

	int64_t now = k_uptime_get();
	bool moved = (dx != 0) || (dy != 0);
	if (moved) {
		data->current_poll_ms = cfg->poll_active_ms;
		data->last_move_time = now;
	} else if (data->current_poll_ms == cfg->poll_active_ms &&
		   (now - data->last_move_time) > (int64_t)cfg->idle_timeout_ms) {
		data->current_poll_ms = cfg->poll_idle_ms;
	}

	k_work_reschedule(dwork, K_MSEC(data->current_poll_ms));
}

static int trackpoint_init(const struct device *dev)
{
	struct trackpoint_data *data = dev->data;
	const struct trackpoint_config *cfg = dev->config;

	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("i2c bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	data->dev = dev;
	data->current_poll_ms = cfg->poll_idle_ms;
	data->last_move_time = k_uptime_get();
	data->dx_offset_q8 = 0;
	data->dy_offset_q8 = 0;
	data->last_buttons = 0;
	data->baseline_seeded = false;

	k_work_init_delayable(&data->work, trackpoint_poll);
	k_work_reschedule(&data->work, K_MSEC(500));

	LOG_INF("hanc trackpoint ready on %s@0x%02x", cfg->bus.bus->name,
		cfg->bus.addr);
	return 0;
}

#define TRACKPOINT_INIT(n)                                                     \
	static struct trackpoint_data trackpoint_data_##n;                     \
	static const struct trackpoint_config trackpoint_cfg_##n = {           \
		.bus = I2C_DT_SPEC_INST_GET(n),                                \
		.poll_idle_ms = DT_INST_PROP(n, poll_interval_idle_ms),        \
		.poll_active_ms = DT_INST_PROP(n, poll_interval_active_ms),    \
		.idle_timeout_ms = DT_INST_PROP(n, idle_timeout_ms),           \
		.dead_zone = DT_INST_PROP(n, dead_zone),                       \
		.max_speed = DT_INST_PROP(n, max_speed),                       \
		.invert_x = DT_INST_PROP(n, invert_x),                         \
		.invert_y = DT_INST_PROP(n, invert_y),                         \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(n, trackpoint_init, NULL,                        \
			      &trackpoint_data_##n, &trackpoint_cfg_##n,       \
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_INIT)
