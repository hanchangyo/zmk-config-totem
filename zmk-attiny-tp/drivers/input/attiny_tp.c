#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT attiny_i2c_tp

LOG_MODULE_REGISTER(attiny_tp, CONFIG_INPUT_LOG_LEVEL);

/* Polling intervals in ms */
#define LOW_POLL_MS      125
#define HIGH_POLL_MS     5
#define IDLE_TIMEOUT_MS  1500

/* Movement filtering */
#define DEAD_ZONE        20
#define FILTER_SHIFT     4   /* 1/16 step IIR for baseline drift */

struct attiny_tp_config {
    struct i2c_dt_spec i2c;
};

struct attiny_tp_data {
    const struct device *dev;
    struct k_work_delayable work;
    uint32_t poll_ms;
    int64_t  last_move_time;
    int32_t  dx_offset_q8;
    int32_t  dy_offset_q8;
    uint8_t  prev_buttons;
};

static void process_buttons(const struct device *dev, uint8_t cur, uint8_t prev)
{
    static const uint16_t codes[] = {
        INPUT_BTN_LEFT, INPUT_BTN_RIGHT, INPUT_BTN_MIDDLE
    };

    for (int i = 0; i < 3; i++) {
        if ((cur ^ prev) & BIT(i)) {
            input_report_key(dev, codes[i], (cur & BIT(i)) ? 1 : 0, true, K_NO_WAIT);
        }
    }
}

static void attiny_tp_poll(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct attiny_tp_data *data = CONTAINER_OF(dwork, struct attiny_tp_data, work);
    const struct device *dev = data->dev;
    const struct attiny_tp_config *cfg = dev->config;

    uint8_t buf[10];
    if (i2c_read_dt(&cfg->i2c, buf, sizeof(buf)) < 0) {
        k_work_reschedule(&data->work, K_MSEC(250));
        return;
    }

    /* Unpack little-endian int16 values from packet */
    int16_t raw_dx = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    int16_t raw_dy = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    uint8_t buttons = buf[9];

    /* Apply Q8 fixed-point drift offset */
    int16_t off_dx = (int16_t)((data->dx_offset_q8 + 128) >> 8);
    int16_t off_dy = (int16_t)((data->dy_offset_q8 + 128) >> 8);
    int16_t comp_dx = raw_dx - off_dx;
    int16_t comp_dy = raw_dy - off_dy;

    /* Update baseline drift estimate only while idle at low poll rate */
    if (data->poll_ms == LOW_POLL_MS && buttons == 0 &&
        abs(comp_dx) < DEAD_ZONE && abs(comp_dy) < DEAD_ZONE) {
        data->dx_offset_q8 += (((int32_t)raw_dx << 8) - data->dx_offset_q8) >> FILTER_SHIFT;
        data->dy_offset_q8 += (((int32_t)raw_dy << 8) - data->dy_offset_q8) >> FILTER_SHIFT;
        comp_dx = raw_dx - (int16_t)((data->dx_offset_q8 + 128) >> 8);
        comp_dy = raw_dy - (int16_t)((data->dy_offset_q8 + 128) >> 8);
    }

    int16_t dx = (abs(comp_dx) >= DEAD_ZONE) ? comp_dx : 0;
    int16_t dy = (abs(comp_dy) >= DEAD_ZONE) ? comp_dy : 0;

    if (dx != 0 || dy != 0) {
        if (dx != 0) {
            input_report_rel(dev, INPUT_REL_X, dx, (dy == 0), K_NO_WAIT);
        }
        if (dy != 0) {
            input_report_rel(dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
        }
    }

    if (buttons != data->prev_buttons) {
        process_buttons(dev, buttons, data->prev_buttons);
        data->prev_buttons = buttons;
    }

    int64_t now = k_uptime_get();
    if (dx != 0 || dy != 0) {
        data->poll_ms = HIGH_POLL_MS;
        data->last_move_time = now;
    } else if (data->poll_ms == HIGH_POLL_MS &&
               (now - data->last_move_time) > IDLE_TIMEOUT_MS) {
        data->poll_ms = LOW_POLL_MS;
    }

    k_work_reschedule(&data->work, K_MSEC(data->poll_ms));
}

static int attiny_tp_init(const struct device *dev)
{
    struct attiny_tp_data *data = dev->data;
    const struct attiny_tp_config *cfg = dev->config;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->dev = dev;
    data->poll_ms = LOW_POLL_MS;
    data->last_move_time = 0;
    data->dx_offset_q8 = 0;
    data->dy_offset_q8 = 0;
    data->prev_buttons = 0;

    k_work_init_delayable(&data->work, attiny_tp_poll);
    k_work_reschedule(&data->work, K_MSEC(500));
    return 0;
}

#define ATTINY_TP_DEFINE(n)                                                    \
    static struct attiny_tp_data attiny_tp_data_##n;                           \
    static const struct attiny_tp_config attiny_tp_config_##n = {              \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                        \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, attiny_tp_init, NULL,                             \
                          &attiny_tp_data_##n, &attiny_tp_config_##n,          \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ATTINY_TP_DEFINE)
