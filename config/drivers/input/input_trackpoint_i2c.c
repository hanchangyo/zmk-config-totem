/*
 * hanc ATTiny-based I2C TrackPoint input driver.
 *
 * Ported from the Arduino test project at
 *   PlatformIO/Projects/xiao nrf52840 trackpoint/src/main.cpp
 *
 * Wire frame (10 bytes):
 *   [0]      seq
 *   [1..2]   dx (int16, little-endian)
 *   [3..4]   dy (int16, little-endian)
 *   [5..6]   x  (int16, little-endian, unused here)
 *   [7..8]   y  (int16, little-endian, unused here)
 *   [9]      buttons (bit0=L, bit1=R, bit2=M)
 */

#define DT_DRV_COMPAT hanc_trackpoint_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <hanc/trackpoint.h>

LOG_MODULE_REGISTER(hanc_trackpoint, CONFIG_INPUT_HANC_TRACKPOINT_I2C_LOG_LEVEL);

#define TP_FRAME_LEN 10
#define TP_Q8_ONE    256
#define TP_Q8_ROUND  128

struct tp_config {
	struct i2c_dt_spec i2c;
	uint16_t low_poll_ms;
	uint16_t high_poll_ms;
	uint16_t idle_timeout_ms;
	uint16_t dead_zone;
	uint8_t  offset_filter_shift;
	bool     invert_x_default;
	bool     invert_y_default;
	bool     swap_xy_default;
	uint16_t sensitivity_default;
	uint16_t sensitivity_step;
	uint16_t sensitivity_min;
	uint16_t sensitivity_max;
};

struct tp_data {
	const struct device *dev;
	struct k_work_delayable work;

	/* Baseline drift estimate in Q8 fixed point. */
	int32_t dx_offset_q8;
	int32_t dy_offset_q8;

	/* Dynamic-poll state. */
	uint16_t current_poll_ms;
	int64_t  last_move_time;

	/* Runtime-tunable state (atomic so it's lock-free wrt the work handler). */
	atomic_t sensitivity_percent;
	atomic_t invert_x;
	atomic_t invert_y;
	atomic_t swap_xy;
};

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

static inline int16_t tp_abs16(int16_t v)
{
	return v < 0 ? (int16_t)-v : v;
}

static inline int16_t tp_clamp_i16(int32_t v)
{
	if (v > INT16_MAX) {
		return INT16_MAX;
	}
	if (v < INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)v;
}

static inline uint16_t tp_clamp_percent(const struct tp_config *cfg, int32_t pct)
{
	if (pct < (int32_t)cfg->sensitivity_min) {
		pct = cfg->sensitivity_min;
	}
	if (pct > (int32_t)cfg->sensitivity_max) {
		pct = cfg->sensitivity_max;
	}
	return (uint16_t)pct;
}

/* Scale by percent with rounding; keeps sign. */
static inline int16_t tp_scale_percent(int16_t v, uint16_t percent)
{
	int32_t s = (int32_t)v * (int32_t)percent;

	if (s >= 0) {
		s = (s + 50) / 100;
	} else {
		s = (s - 50) / 100;
	}
	return tp_clamp_i16(s);
}

/* --------------------------------------------------------------------------
 * Poll work
 * -------------------------------------------------------------------------- */

static void tp_poll_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tp_data *data = CONTAINER_OF(dwork, struct tp_data, work);
	const struct device *dev = data->dev;
	const struct tp_config *cfg = dev->config;

	uint8_t buf[TP_FRAME_LEN];
	int ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));

	if (ret < 0) {
		LOG_WRN("i2c_read_dt failed: %d", ret);
		/* Back off a little on bus errors. */
		k_work_reschedule(&data->work, K_MSEC(250));
		return;
	}

	int16_t raw_dx = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
	int16_t raw_dy = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
	uint8_t buttons = buf[9];

	/* Current integer offset estimate (rounded from Q8). */
	int16_t offset_dx = (int16_t)((data->dx_offset_q8 + TP_Q8_ROUND) >> 8);
	int16_t offset_dy = (int16_t)((data->dy_offset_q8 + TP_Q8_ROUND) >> 8);
	int16_t comp_dx = raw_dx - offset_dx;
	int16_t comp_dy = raw_dy - offset_dy;

	/* Learn baseline drift only while polling slowly and near idle. */
	if (data->current_poll_ms == cfg->low_poll_ms && buttons == 0 &&
	    tp_abs16(comp_dx) < (int16_t)cfg->dead_zone &&
	    tp_abs16(comp_dy) < (int16_t)cfg->dead_zone) {

		int32_t target_dx_q8 = ((int32_t)raw_dx) << 8;
		int32_t target_dy_q8 = ((int32_t)raw_dy) << 8;
		data->dx_offset_q8 +=
			(target_dx_q8 - data->dx_offset_q8) >> cfg->offset_filter_shift;
		data->dy_offset_q8 +=
			(target_dy_q8 - data->dy_offset_q8) >> cfg->offset_filter_shift;

		offset_dx = (int16_t)((data->dx_offset_q8 + TP_Q8_ROUND) >> 8);
		offset_dy = (int16_t)((data->dy_offset_q8 + TP_Q8_ROUND) >> 8);
		comp_dx = raw_dx - offset_dx;
		comp_dy = raw_dy - offset_dy;
	}

	int16_t dx = comp_dx;
	int16_t dy = comp_dy;

	if (tp_abs16(dx) < (int16_t)cfg->dead_zone &&
	    tp_abs16(dy) < (int16_t)cfg->dead_zone) {
		dx = 0;
		dy = 0;
	}

	bool moved = (dx != 0) || (dy != 0);

	if (moved) {
		/* Apply runtime transforms: swap -> invert -> scale. */
		if (atomic_get(&data->swap_xy)) {
			int16_t t = dx;
			dx = dy;
			dy = t;
		}
		if (atomic_get(&data->invert_x)) {
			dx = -dx;
		}
		if (atomic_get(&data->invert_y)) {
			dy = -dy;
		}

		uint16_t sens = (uint16_t)atomic_get(&data->sensitivity_percent);
		dx = tp_scale_percent(dx, sens);
		dy = tp_scale_percent(dy, sens);

		if (dx != 0) {
			input_report_rel(dev, INPUT_REL_X, dx, false, K_NO_WAIT);
		}
		input_report_rel(dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
	}

	/* TODO: forward `buttons` as INPUT_BTN_0/1/2 when we enable mouse buttons. */
	ARG_UNUSED(buttons);

	/* Dynamic polling. */
	int64_t now = k_uptime_get();

	if (moved) {
		if (data->current_poll_ms != cfg->high_poll_ms) {
			LOG_DBG("motion: poll -> %u ms", cfg->high_poll_ms);
		}
		data->current_poll_ms = cfg->high_poll_ms;
		data->last_move_time = now;
	} else if (data->current_poll_ms == cfg->high_poll_ms &&
		   (now - data->last_move_time) > cfg->idle_timeout_ms) {
		LOG_DBG("idle: poll -> %u ms", cfg->low_poll_ms);
		data->current_poll_ms = cfg->low_poll_ms;
	}

	k_work_reschedule(&data->work, K_MSEC(data->current_poll_ms));
}

/* --------------------------------------------------------------------------
 * Public runtime-tuning API
 * -------------------------------------------------------------------------- */

int hanc_trackpoint_set_sensitivity(const struct device *dev, uint16_t percent)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct tp_config *cfg = dev->config;
	struct tp_data *data = dev->data;
	uint16_t v = tp_clamp_percent(cfg, (int32_t)percent);

	atomic_set(&data->sensitivity_percent, (atomic_val_t)v);
	LOG_INF("sensitivity -> %u%%", v);
	return 0;
}

int hanc_trackpoint_adjust_sensitivity(const struct device *dev, int16_t delta_percent)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct tp_config *cfg = dev->config;
	struct tp_data *data = dev->data;
	int32_t cur = (int32_t)atomic_get(&data->sensitivity_percent);
	uint16_t v = tp_clamp_percent(cfg, cur + delta_percent);

	atomic_set(&data->sensitivity_percent, (atomic_val_t)v);
	LOG_INF("sensitivity %ld -> %u%%", (long)cur, v);
	return 0;
}

int hanc_trackpoint_step_sensitivity(const struct device *dev, int direction)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct tp_config *cfg = dev->config;
	int16_t step = (int16_t)cfg->sensitivity_step;

	if (direction > 0) {
		return hanc_trackpoint_adjust_sensitivity(dev, +step);
	} else if (direction < 0) {
		return hanc_trackpoint_adjust_sensitivity(dev, -step);
	}
	return 0;
}

uint16_t hanc_trackpoint_get_sensitivity(const struct device *dev)
{
	if (dev == NULL) {
		return 0;
	}
	struct tp_data *data = dev->data;

	return (uint16_t)atomic_get(&data->sensitivity_percent);
}

int hanc_trackpoint_set_invert_x(const struct device *dev, bool invert)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	struct tp_data *data = dev->data;

	atomic_set(&data->invert_x, (atomic_val_t)(invert ? 1 : 0));
	LOG_INF("invert_x -> %d", invert);
	return 0;
}

int hanc_trackpoint_set_invert_y(const struct device *dev, bool invert)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	struct tp_data *data = dev->data;

	atomic_set(&data->invert_y, (atomic_val_t)(invert ? 1 : 0));
	LOG_INF("invert_y -> %d", invert);
	return 0;
}

int hanc_trackpoint_toggle_invert_x(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	struct tp_data *data = dev->data;

	atomic_val_t old = atomic_get(&data->invert_x);
	atomic_set(&data->invert_x, old ? 0 : 1);
	LOG_INF("invert_x toggled -> %d", !old);
	return 0;
}

int hanc_trackpoint_toggle_invert_y(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	struct tp_data *data = dev->data;

	atomic_val_t old = atomic_get(&data->invert_y);
	atomic_set(&data->invert_y, old ? 0 : 1);
	LOG_INF("invert_y toggled -> %d", !old);
	return 0;
}

int hanc_trackpoint_set_swap_xy(const struct device *dev, bool swap)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	struct tp_data *data = dev->data;

	atomic_set(&data->swap_xy, (atomic_val_t)(swap ? 1 : 0));
	LOG_INF("swap_xy -> %d", swap);
	return 0;
}

int hanc_trackpoint_toggle_swap_xy(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	struct tp_data *data = dev->data;

	atomic_val_t old = atomic_get(&data->swap_xy);
	atomic_set(&data->swap_xy, old ? 0 : 1);
	LOG_INF("swap_xy toggled -> %d", !old);
	return 0;
}

int hanc_trackpoint_reset_tuning(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct tp_config *cfg = dev->config;
	struct tp_data *data = dev->data;

	atomic_set(&data->sensitivity_percent, (atomic_val_t)cfg->sensitivity_default);
	atomic_set(&data->invert_x, (atomic_val_t)(cfg->invert_x_default ? 1 : 0));
	atomic_set(&data->invert_y, (atomic_val_t)(cfg->invert_y_default ? 1 : 0));
	atomic_set(&data->swap_xy, (atomic_val_t)(cfg->swap_xy_default ? 1 : 0));
	LOG_INF("tuning reset to DT defaults");
	return 0;
}

/* --------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------- */

static int tp_init(const struct device *dev)
{
	const struct tp_config *cfg = dev->config;
	struct tp_data *data = dev->data;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	data->dev = dev;
	data->dx_offset_q8 = 0;
	data->dy_offset_q8 = 0;
	data->current_poll_ms = cfg->low_poll_ms;
	data->last_move_time = 0;

	atomic_set(&data->sensitivity_percent, (atomic_val_t)cfg->sensitivity_default);
	atomic_set(&data->invert_x, (atomic_val_t)(cfg->invert_x_default ? 1 : 0));
	atomic_set(&data->invert_y, (atomic_val_t)(cfg->invert_y_default ? 1 : 0));
	atomic_set(&data->swap_xy, (atomic_val_t)(cfg->swap_xy_default ? 1 : 0));

	k_work_init_delayable(&data->work, tp_poll_work_handler);
	k_work_reschedule(&data->work, K_MSEC(cfg->low_poll_ms));

	LOG_INF("hanc trackpoint ready @ 0x%02x on %s (sens=%u%%)",
		cfg->i2c.addr, cfg->i2c.bus->name, cfg->sensitivity_default);
	return 0;
}

/* --------------------------------------------------------------------------
 * DT instantiation
 * -------------------------------------------------------------------------- */

#define HANC_TP_INIT(n)                                                                        \
	static struct tp_data tp_data_##n;                                                     \
	static const struct tp_config tp_cfg_##n = {                                           \
		.i2c                 = I2C_DT_SPEC_INST_GET(n),                                \
		.low_poll_ms         = DT_INST_PROP(n, low_poll_ms),                           \
		.high_poll_ms        = DT_INST_PROP(n, high_poll_ms),                          \
		.idle_timeout_ms     = DT_INST_PROP(n, idle_timeout_ms),                       \
		.dead_zone           = DT_INST_PROP(n, dead_zone),                             \
		.offset_filter_shift = DT_INST_PROP(n, offset_filter_shift),                   \
		.invert_x_default    = DT_INST_PROP(n, invert_x),                              \
		.invert_y_default    = DT_INST_PROP(n, invert_y),                              \
		.swap_xy_default     = DT_INST_PROP(n, swap_xy),                               \
		.sensitivity_default = DT_INST_PROP(n, sensitivity_percent),                   \
		.sensitivity_step    = DT_INST_PROP(n, sensitivity_step_percent),              \
		.sensitivity_min     = DT_INST_PROP(n, sensitivity_min_percent),               \
		.sensitivity_max     = DT_INST_PROP(n, sensitivity_max_percent),               \
	};                                                                                     \
	DEVICE_DT_INST_DEFINE(n, tp_init, NULL, &tp_data_##n, &tp_cfg_##n,                     \
			      POST_KERNEL,                                                     \
			      CONFIG_INPUT_HANC_TRACKPOINT_I2C_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(HANC_TP_INIT)
