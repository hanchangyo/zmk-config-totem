/*
 * Minimal ATTiny I2C TrackPoint driver for ZMK — DEBUG BUILD.
 *
 * This is intentionally a near-verbatim port of the Arduino reference
 * sketch in xiao nrf52840 trackpoint/src/main.cpp. Its only jobs are:
 *   1. Poll the ATTiny over I2C and read 10 bytes.
 *   2. Log the raw frame in the exact same format as the Arduino sketch
 *      so logs from both can be diffed directly.
 *   3. Switch between 10 ms "active" polling and 125 ms "idle" polling
 *      based on the same dead-zone rule as the sketch.
 *
 * HID mouse output is intentionally NOT emitted here — the input listener
 * node is also disabled in the overlay — so we can watch the raw ATTiny
 * stream under ZMK without the cursor moving on the host.
 *
 * Protocol recap (ATTiny 0x20, 10 bytes little-endian):
 *   buf[0]    : seq counter (wraps 0..255)
 *   buf[1-2]  : int16_t raw_dx
 *   buf[3-4]  : int16_t raw_dy
 *   buf[5-6]  : uint16_t raw_x (0..1023 ADC)
 *   buf[7-8]  : uint16_t raw_y (0..1023 ADC)
 *   buf[9]    : button bitmap
 *
 * Invariant: raw_dx == (int16_t)raw_x - 512, same for Y.
 */

#define DT_DRV_COMPAT zmk_input_trackpoint

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(input_trackpoint, CONFIG_INPUT_TRACKPOINT_LOG_LEVEL);

#define TP_READ_LEN     10

/* Mirror of the Arduino sketch constants. */
#define LOW_POLL_MS     125
#define HIGH_POLL_MS    10
#define IDLE_TIMEOUT_MS 1500
#define DEAD_ZONE       30
#define OFFSET          (-20) /* applied as `dx -= OFFSET`, i.e. dx += 20 */

struct trackpoint_config {
    struct i2c_dt_spec i2c;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work_delayable work;

    uint32_t current_poll_ms;
    int64_t  last_move_time;
};

static int trackpoint_poll_once(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    uint8_t buf[TP_READ_LEN];
    int ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    uint8_t  seq     = buf[0];
    int16_t  dx      = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    int16_t  dy      = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    uint16_t raw_x   = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
    uint16_t raw_y   = (uint16_t)buf[7] | ((uint16_t)buf[8] << 8);
    uint8_t  buttons = buf[9];

    dx -= OFFSET;
    dy -= OFFSET;

    /* Same single-line readout the Arduino sketch produces. */
    LOG_INF("poll_ms=%u  seq=%u  dx=%d  dy=%d  raw_x=%u  raw_y=%u  buttons=%u",
            data->current_poll_ms, seq, dx, dy, raw_x, raw_y, buttons);

    if (abs((int)dx) < DEAD_ZONE && abs((int)dy) < DEAD_ZONE) {
        dx = 0;
        dy = 0;
    }

    int64_t now = k_uptime_get();

    if (abs((int)dx) > DEAD_ZONE || abs((int)dy) > DEAD_ZONE) {
        if (data->current_poll_ms == LOW_POLL_MS) {
            LOG_INF("--- MOVEMENT: Switching to %u ms poll ---", HIGH_POLL_MS);
        }
        data->current_poll_ms = HIGH_POLL_MS;
        data->last_move_time  = now;
    } else {
        if (data->current_poll_ms == HIGH_POLL_MS &&
            (now - data->last_move_time) > (int64_t)IDLE_TIMEOUT_MS) {
            LOG_INF("--- MOVEMENT: Switching to %u ms poll ---", LOW_POLL_MS);
            data->current_poll_ms = LOW_POLL_MS;
        }
    }

    return 0;
}

static void trackpoint_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct trackpoint_data *data =
        CONTAINER_OF(dwork, struct trackpoint_data, work);
    const struct device *dev = data->dev;

    int ret = trackpoint_poll_once(dev);
    if (ret < 0) {
        LOG_WRN("I2C read failed: %d", ret);
    }

    k_work_reschedule(&data->work, K_MSEC(data->current_poll_ms));
}

static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
        return -ENODEV;
    }

    data->dev             = dev;
    data->current_poll_ms = LOW_POLL_MS;
    data->last_move_time  = 0;

    k_work_init_delayable(&data->work, trackpoint_work_handler);
    k_work_schedule(&data->work, K_MSEC(500));

    LOG_INF("trackpoint DEBUG init on %s @ 0x%02x (minimal, HID disabled)",
            cfg->i2c.bus->name, cfg->i2c.addr);
    return 0;
}

#define TP_INST(n)                                                                 \
    static struct trackpoint_data trackpoint_data_##n;                             \
    static const struct trackpoint_config trackpoint_cfg_##n = {                   \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                            \
    };                                                                             \
    DEVICE_DT_INST_DEFINE(n, trackpoint_init, NULL,                                \
                          &trackpoint_data_##n, &trackpoint_cfg_##n,               \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TP_INST)
