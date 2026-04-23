/*
 * ATTiny I2C TrackPoint driver for ZMK.
 *
 * Behavior mirrors the Arduino reference implementation, with a few
 * additions needed once the stream is fed into an HID mouse:
 *   - Read 10 bytes from the ATTiny (default addr 0x20):
 *       buf[0]   : sequence counter (wraps 0..255)
 *       buf[1-2] : int16_t raw_dx (little-endian)
 *       buf[3-4] : int16_t raw_dy
 *       buf[5-6] : int16_t raw_x   (unused here)
 *       buf[7-8] : int16_t raw_y   (unused here)
 *       buf[9]   : button bitmap (bit0/1/2 -> BTN_0/1/2)
 *   - Deduplicate samples using `seq` so over-polling doesn't amplify
 *     motion (the ATTiny updates at its own rate, independent of ours).
 *   - Continuously learn a Q8 baseline whenever the compensated value is
 *     inside the dead zone and no buttons are held. This prevents DC
 *     bias from spilling into HID as sustained drift / "teleports".
 *   - Apply a configurable right-shift (sensitivity-shift) to scale the
 *     output; fractional remainders are carried across polls so small
 *     sustained motions aren't lost.
 *   - Dynamic poll rate: fast while moving / buttons held, slow when idle.
 *   - Emit INPUT_REL_X / INPUT_REL_Y / INPUT_BTN_* events to this device.
 *     A separate zmk,input-listener node forwards them into HID.
 */

#define DT_DRV_COMPAT zmk_input_trackpoint

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(input_trackpoint, CONFIG_INPUT_TRACKPOINT_LOG_LEVEL);

#define OFFSET_FILTER_SHIFT 4 /* Q8 baseline learning rate: 1/16 per sample */
#define TP_READ_LEN         10

struct trackpoint_config {
    struct i2c_dt_spec i2c;
    uint32_t poll_period_ms;
    uint32_t idle_poll_period_ms;
    uint32_t idle_timeout_ms;
    uint16_t dead_zone;
    uint8_t  sensitivity_shift;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work_delayable work;

    int64_t last_move_time;
    bool active; /* true => using fast poll rate */

    int32_t dx_offset_q8;
    int32_t dy_offset_q8;

    /* Fractional remainders kept across polls so sub-integer motion
     * accumulates instead of being clipped to zero. */
    int16_t dx_remainder;
    int16_t dy_remainder;

    uint8_t last_buttons;
    uint8_t last_seq;
    bool    seq_valid; /* last_seq has been populated at least once */
};

/* Apply sensitivity right-shift with remainder carry. */
static inline int16_t tp_scale(int16_t raw, int16_t *remainder, uint8_t shift) {
    if (shift == 0) {
        return raw;
    }

    int32_t total = (int32_t)raw + (int32_t)*remainder;
    int32_t scaled;

    if (total >= 0) {
        scaled = total >> shift;
        *remainder = (int16_t)(total - (scaled << shift));
    } else {
        /* Arithmetic right-shift on negative is implementation-defined in
         * theory; use an explicit division that rounds toward zero so the
         * remainder sign stays well-defined. */
        int32_t neg = -total;
        int32_t neg_scaled = neg >> shift;
        scaled = -neg_scaled;
        *remainder = (int16_t)(total - (scaled << shift));
    }

    return (int16_t)scaled;
}

static int trackpoint_poll_once(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    uint8_t buf[TP_READ_LEN];
    int ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    uint8_t seq     = buf[0];
    int16_t raw_dx  = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    int16_t raw_dy  = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    uint8_t buttons = buf[9];

    bool fresh_sample = !data->seq_valid || (seq != data->last_seq);
    data->last_seq = seq;
    data->seq_valid = true;

    int16_t off_dx  = (int16_t)((data->dx_offset_q8 + 128) >> 8);
    int16_t off_dy  = (int16_t)((data->dy_offset_q8 + 128) >> 8);
    int16_t comp_dx = raw_dx - off_dx;
    int16_t comp_dy = raw_dy - off_dy;

    /* Continuous baseline self-calibration: whenever we're within the
     * dead zone and no buttons are held, nudge the stored baseline toward
     * the current raw value. Removing the previous "!active" gate keeps
     * DC bias from accumulating while the user is using the stick, which
     * was the main cause of spontaneous pointer drift/teleportation. */
    if (buttons == 0 &&
        abs(comp_dx) < (int)cfg->dead_zone &&
        abs(comp_dy) < (int)cfg->dead_zone) {

        int32_t tgt_dx_q8 = ((int32_t)raw_dx) << 8;
        int32_t tgt_dy_q8 = ((int32_t)raw_dy) << 8;

        data->dx_offset_q8 += (tgt_dx_q8 - data->dx_offset_q8) >> OFFSET_FILTER_SHIFT;
        data->dy_offset_q8 += (tgt_dy_q8 - data->dy_offset_q8) >> OFFSET_FILTER_SHIFT;

        off_dx  = (int16_t)((data->dx_offset_q8 + 128) >> 8);
        off_dy  = (int16_t)((data->dy_offset_q8 + 128) >> 8);
        comp_dx = raw_dx - off_dx;
        comp_dy = raw_dy - off_dy;

        /* Also slowly bleed off any leftover scaling remainders while
         * idle, so they can't accumulate into a surprise jump. */
        data->dx_remainder = 0;
        data->dy_remainder = 0;
    }

    int16_t dx = comp_dx;
    int16_t dy = comp_dy;
    if (abs(dx) < (int)cfg->dead_zone && abs(dy) < (int)cfg->dead_zone) {
        dx = 0;
        dy = 0;
    }

    dx = tp_scale(dx, &data->dx_remainder, cfg->sensitivity_shift);
    dy = tp_scale(dy, &data->dy_remainder, cfg->sensitivity_shift);

    if (IS_ENABLED(CONFIG_INPUT_TRACKPOINT_SWAP_XY)) {
        int16_t tmp = dx;
        dx = dy;
        dy = tmp;
    }
    if (IS_ENABLED(CONFIG_INPUT_TRACKPOINT_INVERT_X)) {
        dx = -dx;
    }
    if (IS_ENABLED(CONFIG_INPUT_TRACKPOINT_INVERT_Y)) {
        dy = -dy;
    }

    bool moved = (dx != 0) || (dy != 0);

    /* Only inject motion into HID if this is a genuinely new sample from
     * the ATTiny; duplicated readings (we polled faster than the slave
     * updated) would otherwise double-count every motion event. */
    if (moved && fresh_sample) {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }

    uint8_t changed = buttons ^ data->last_buttons;
    if (changed) {
        if (changed & BIT(0)) {
            input_report_key(dev, INPUT_BTN_0,
                             (buttons & BIT(0)) ? 1 : 0, true, K_NO_WAIT);
        }
        if (changed & BIT(1)) {
            input_report_key(dev, INPUT_BTN_1,
                             (buttons & BIT(1)) ? 1 : 0, true, K_NO_WAIT);
        }
        if (changed & BIT(2)) {
            input_report_key(dev, INPUT_BTN_2,
                             (buttons & BIT(2)) ? 1 : 0, true, K_NO_WAIT);
        }
        data->last_buttons = buttons;
    }

    int64_t now = k_uptime_get();
    if (moved || buttons) {
        data->active = true;
        data->last_move_time = now;
    } else if (data->active && (now - data->last_move_time > (int64_t)cfg->idle_timeout_ms)) {
        data->active = false;
    }

    return 0;
}

static void trackpoint_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct trackpoint_data *data =
        CONTAINER_OF(dwork, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    int ret = trackpoint_poll_once(dev);
    if (ret < 0) {
        LOG_WRN("I2C read failed: %d", ret);
    }

    uint32_t period = data->active ? cfg->poll_period_ms : cfg->idle_poll_period_ms;
    k_work_reschedule(&data->work, K_MSEC(period));
}

static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
        return -ENODEV;
    }

    data->dev            = dev;
    data->active         = false;
    data->last_move_time = 0;
    data->dx_offset_q8   = 0;
    data->dy_offset_q8   = 0;
    data->dx_remainder   = 0;
    data->dy_remainder   = 0;
    data->last_buttons   = 0;
    data->last_seq       = 0;
    data->seq_valid      = false;

    k_work_init_delayable(&data->work, trackpoint_work_handler);
    /* Delay the first poll a bit so the ATTiny has time to come up. */
    k_work_schedule(&data->work, K_MSEC(500));

    LOG_INF("trackpoint init on %s @ 0x%02x "
            "(fast=%ums idle=%ums dz=%u sh=%u invx=%d invy=%d swap=%d)",
            cfg->i2c.bus->name, cfg->i2c.addr,
            cfg->poll_period_ms, cfg->idle_poll_period_ms, cfg->dead_zone,
            cfg->sensitivity_shift,
            IS_ENABLED(CONFIG_INPUT_TRACKPOINT_INVERT_X),
            IS_ENABLED(CONFIG_INPUT_TRACKPOINT_INVERT_Y),
            IS_ENABLED(CONFIG_INPUT_TRACKPOINT_SWAP_XY));

    return 0;
}

#define TP_INST(n)                                                                 \
    static struct trackpoint_data trackpoint_data_##n;                             \
    static const struct trackpoint_config trackpoint_cfg_##n = {                   \
        .i2c                 = I2C_DT_SPEC_INST_GET(n),                            \
        .poll_period_ms      = DT_INST_PROP(n, poll_period_ms),                    \
        .idle_poll_period_ms = DT_INST_PROP(n, idle_poll_period_ms),               \
        .idle_timeout_ms     = DT_INST_PROP(n, idle_timeout_ms),                   \
        .dead_zone           = DT_INST_PROP(n, dead_zone),                         \
        .sensitivity_shift   = DT_INST_PROP(n, sensitivity_shift),                 \
    };                                                                             \
    DEVICE_DT_INST_DEFINE(n, trackpoint_init, NULL,                                \
                          &trackpoint_data_##n, &trackpoint_cfg_##n,               \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TP_INST)
