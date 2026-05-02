/*
 * ATTiny I2C TrackPoint driver for ZMK.
 *
 * Pipeline per poll:
 *   1. Read 10 bytes from the ATTiny (seq, raw_dx, raw_dy, raw_x, raw_y,
 *      buttons).
 *   2. Apply the fixed OFFSET compensation that the Arduino reference
 *      sketch uses (empirical ~-20 DC bias on this hardware).
 *   3. Dead-zone: samples with |dx| < dead-zone AND |dy| < dead-zone are
 *      reported as zero motion.
 *   4. Sensitivity right-shift with fractional remainder carry, so slow
 *      drags still accumulate pixel-by-pixel under the divisor.
 *   5. Optional swap-xy / invert-x / invert-y (Kconfig).
 *   6. Emit INPUT_REL_X / INPUT_REL_Y / INPUT_BTN_* to this device; a
 *      zmk,input-listener forwards them into HID.
 *
 * Scheduling uses a simple `k_work_reschedule(K_MSEC(current_poll_ms))`
 * after each poll. That makes the actual cycle time slightly longer
 * than the configured value (interval + I2C/handler runtime), which is
 * fine: the ATTiny's own ADC pacing is the real cadence anyway, and an
 * earlier attempt to enforce a fixed absolute deadline introduced
 * periodic cursor jumps when the catch-up rescheduling collided with
 * the active/idle state machine.
 *
 * Note: there is intentionally NO software spike rejector or EMA filter
 * here. Earlier branches added those to mask BLE-induced ADC glitches
 * and active-motion jitter. Once the noise was traced to the analog
 * front end and fixed in hardware (local decoupling on ATTiny VDD,
 * ferrite + bulk on the 3V3 rail, RC low-pass on the ADC inputs) the
 * software workarounds became unnecessary. If new noise modes appear,
 * re-introduce them rather than tweaking this baseline.
 *
 * Protocol recap (ATTiny 0x20, 10 bytes little-endian):
 *   buf[0]    : seq counter (wraps 0..255)
 *   buf[1-2]  : int16_t raw_dx
 *   buf[3-4]  : int16_t raw_dy
 *   buf[5-6]  : uint16_t raw_x (0..1023 ADC)
 *   buf[7-8]  : uint16_t raw_y (0..1023 ADC)
 *   buf[9]    : button bitmap (bit0..2 -> BTN_0..2)
 *
 * Invariant: raw_dx == (int16_t)raw_x - 512, same for Y.
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

#define TP_READ_LEN     10

/* Empirical DC bias from the Arduino reference sketch; applied as
 * `dx -= OFFSET`, i.e. dx += 20. Stays here as a #define because it
 * tracks a property of the ATTiny firmware, not the keymap. */
#define OFFSET          (-20)

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

    uint32_t current_poll_ms;
    int64_t  last_move_time;

    uint8_t  last_buttons;

    /* Fractional remainders kept across polls so the sensitivity shift
     * doesn't silently truncate slow sustained motion to zero. */
    int16_t  dx_remainder;
    int16_t  dy_remainder;
};

/* Right-shift with remainder carry. shift==0 is a pass-through. */
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
        /* Round toward zero so the carried remainder sign stays
         * well-defined regardless of the compiler's behavior for
         * arithmetic right-shift on negatives. */
        int32_t neg = -total;
        int32_t neg_scaled = neg >> shift;
        scaled = -neg_scaled;
        *remainder = (int16_t)(total - (scaled << shift));
    }

    return (int16_t)scaled;
}

static void tp_emit_motion(const struct device *dev, int16_t dx, int16_t dy) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    if (abs((int)dx) < (int)cfg->dead_zone &&
        abs((int)dy) < (int)cfg->dead_zone) {
        /* Bleed any leftover scaling remainder so it can't accumulate
         * into a surprise jump on the next push. */
        data->dx_remainder = 0;
        data->dy_remainder = 0;
        return;
    }

    int16_t out_dx = tp_scale(dx, &data->dx_remainder, cfg->sensitivity_shift);
    int16_t out_dy = tp_scale(dy, &data->dy_remainder, cfg->sensitivity_shift);

    if (IS_ENABLED(CONFIG_INPUT_TRACKPOINT_SWAP_XY)) {
        int16_t tmp = out_dx;
        out_dx = out_dy;
        out_dy = tmp;
    }
    if (IS_ENABLED(CONFIG_INPUT_TRACKPOINT_INVERT_X)) {
        out_dx = -out_dx;
    }
    if (IS_ENABLED(CONFIG_INPUT_TRACKPOINT_INVERT_Y)) {
        out_dy = -out_dy;
    }

    if (out_dx == 0 && out_dy == 0) {
        return;
    }

    input_report_rel(dev, INPUT_REL_X, out_dx, false, K_NO_WAIT);
    input_report_rel(dev, INPUT_REL_Y, out_dy, true,  K_NO_WAIT);
}

static void tp_emit_buttons(const struct device *dev, uint8_t buttons) {
    struct trackpoint_data *data = dev->data;
    uint8_t changed = buttons ^ data->last_buttons;

    if (!changed) {
        return;
    }

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

    LOG_DBG("poll_ms=%u  seq=%u  dx=%d  dy=%d  raw_x=%u  raw_y=%u  buttons=%u",
            data->current_poll_ms, seq, dx, dy, raw_x, raw_y, buttons);

    tp_emit_motion(dev, dx, dy);
    tp_emit_buttons(dev, buttons);

    bool above_dz = (abs((int)dx) > (int)cfg->dead_zone) ||
                    (abs((int)dy) > (int)cfg->dead_zone);
    bool is_motion = above_dz || (buttons != 0);

    int64_t now = k_uptime_get();
    if (is_motion) {
        if (data->current_poll_ms == cfg->idle_poll_period_ms) {
            LOG_INF("--- MOVEMENT: switching to %u ms poll ---",
                    cfg->poll_period_ms);
        }
        data->current_poll_ms = cfg->poll_period_ms;
        data->last_move_time  = now;
    } else if (data->current_poll_ms == cfg->poll_period_ms &&
               (now - data->last_move_time) >
                   (int64_t)cfg->idle_timeout_ms) {
        LOG_INF("--- IDLE: switching to %u ms poll ---",
                cfg->idle_poll_period_ms);
        data->current_poll_ms = cfg->idle_poll_period_ms;
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
    data->current_poll_ms = cfg->idle_poll_period_ms;
    data->last_move_time  = 0;
    data->last_buttons    = 0;
    data->dx_remainder    = 0;
    data->dy_remainder    = 0;

    k_work_init_delayable(&data->work, trackpoint_work_handler);
    k_work_schedule(&data->work, K_MSEC(500));

    LOG_INF("trackpoint init on %s @ 0x%02x "
            "(active=%u ms idle=%u ms dz=%u sh=%u "
            "invx=%d invy=%d swap=%d)",
            cfg->i2c.bus->name, cfg->i2c.addr,
            cfg->poll_period_ms, cfg->idle_poll_period_ms,
            cfg->dead_zone, cfg->sensitivity_shift,
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
