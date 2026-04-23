/*
 * Copyright (c) 2026 hanc
 * SPDX-License-Identifier: MIT
 *
 * Driver for the ATTiny-based I2C trackpoint co-processor on the Totem
 * keyboard. Ported from the Arduino prototype in
 *   xiao nrf52840 trackpoint/src/main.cpp
 * keeping the same 10-byte packet layout, dead-zone, Q8 baseline filter,
 * and dynamic 125 ms idle / 5 ms active poll cadence.
 */

#define DT_DRV_COMPAT hanc_trackpoint_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(hanc_trackpoint_i2c, CONFIG_INPUT_HANC_TRACKPOINT_I2C_LOG_LEVEL);

#define TP_PACKET_LEN    10
#define TP_WQ_PRIORITY   5   /* preemptive, above BLE work */
#define TP_STALE_FACTOR  3   /* skip HID if elapsed > N * expected interval */

static K_THREAD_STACK_DEFINE(tp_wq_stack,
                             CONFIG_INPUT_HANC_TRACKPOINT_I2C_THREAD_STACK_SIZE);
static struct k_work_q tp_wq;
static bool tp_wq_started;

struct tp_config {
    struct i2c_dt_spec bus;
    uint16_t low_poll_ms;
    uint16_t high_poll_ms;
    uint16_t idle_timeout_ms;
    uint16_t dead_zone;
    uint8_t  offset_filter_shift;
    uint8_t  scale_shift;
    int16_t  max_delta;
    bool     invert_x;
    bool     invert_y;
};

struct tp_data {
    const struct device *dev;
    struct k_work_delayable poll_work;

    int32_t dx_offset_q8;
    int32_t dy_offset_q8;

    uint32_t last_move_time;
    uint32_t last_poll_time;
    uint16_t current_poll_ms;
};

static void tp_poll_work(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tp_data *data = CONTAINER_OF(dwork, struct tp_data, poll_work);
    const struct device *dev = data->dev;
    const struct tp_config *cfg = dev->config;

    uint32_t now = k_uptime_get_32();
    uint32_t elapsed = now - data->last_poll_time;
    bool stale_gap = elapsed > (uint32_t)data->current_poll_ms * TP_STALE_FACTOR;
    data->last_poll_time = now;

    uint8_t buf[TP_PACKET_LEN];
    int ret = i2c_read_dt(&cfg->bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_WRN("i2c_read_dt failed: %d", ret);
        k_work_reschedule_for_queue(&tp_wq, &data->poll_work,
                                    K_MSEC(cfg->low_poll_ms));
        return;
    }

    /* uint8_t seq = buf[0]; — unused but kept for parity with the prototype */
    int16_t raw_dx = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    int16_t raw_dy = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    /* raw_x / raw_y (buf[5..8]) are absolute positions; unused here. */
    uint8_t buttons = buf[9];

    int16_t offset_dx = (int16_t)((data->dx_offset_q8 + 128) >> 8);
    int16_t offset_dy = (int16_t)((data->dy_offset_q8 + 128) >> 8);
    int16_t comp_dx = raw_dx - offset_dx;
    int16_t comp_dy = raw_dy - offset_dy;

    /* Learn baseline drift only while idle (low-poll, no buttons, in dead-zone). */
    if (data->current_poll_ms == cfg->low_poll_ms && buttons == 0 &&
        abs(comp_dx) < cfg->dead_zone && abs(comp_dy) < cfg->dead_zone) {
        int32_t target_dx_q8 = ((int32_t)raw_dx) << 8;
        int32_t target_dy_q8 = ((int32_t)raw_dy) << 8;
        data->dx_offset_q8 += (target_dx_q8 - data->dx_offset_q8) >> cfg->offset_filter_shift;
        data->dy_offset_q8 += (target_dy_q8 - data->dy_offset_q8) >> cfg->offset_filter_shift;

        offset_dx = (int16_t)((data->dx_offset_q8 + 128) >> 8);
        offset_dy = (int16_t)((data->dy_offset_q8 + 128) >> 8);
        comp_dx = raw_dx - offset_dx;
        comp_dy = raw_dy - offset_dy;
    }

    int16_t dx = comp_dx;
    int16_t dy = comp_dy;
    if (abs(dx) < cfg->dead_zone && abs(dy) < cfg->dead_zone) {
        dx = 0;
        dy = 0;
    }

    int32_t out_dx = dx;
    int32_t out_dy = dy;
    if (cfg->scale_shift) {
        out_dx >>= cfg->scale_shift;
        out_dy >>= cfg->scale_shift;
    }
    if (cfg->invert_x) { out_dx = -out_dx; }
    if (cfg->invert_y) { out_dy = -out_dy; }
    out_dx = CLAMP(out_dx, -cfg->max_delta, cfg->max_delta);
    out_dy = CLAMP(out_dy, -cfg->max_delta, cfg->max_delta);

    if (stale_gap) {
        LOG_WRN("stale sample: elapsed %u ms vs expected %u ms — dropping HID",
                elapsed, data->current_poll_ms);
    } else if (out_dx != 0 || out_dy != 0) {
        input_report_rel(dev, INPUT_REL_X, out_dx, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, out_dy, true,  K_NO_WAIT);
    }

    /* Dynamic polling: jump to high-poll on motion, fall back after idle timeout. */
    if (abs(dx) > cfg->dead_zone || abs(dy) > cfg->dead_zone) {
        if (data->current_poll_ms == cfg->low_poll_ms) {
            LOG_INF("movement: switching to %u ms poll", cfg->high_poll_ms);
        }
        data->current_poll_ms = cfg->high_poll_ms;
        data->last_move_time = now;
    } else if (data->current_poll_ms == cfg->high_poll_ms &&
               (now - data->last_move_time) > cfg->idle_timeout_ms) {
        LOG_INF("idle: switching to %u ms poll", cfg->low_poll_ms);
        data->current_poll_ms = cfg->low_poll_ms;
    }

    k_work_reschedule_for_queue(&tp_wq, &data->poll_work,
                                K_MSEC(data->current_poll_ms));
}

static int tp_init(const struct device *dev)
{
    const struct tp_config *cfg = dev->config;
    struct tp_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->bus)) {
        LOG_ERR("i2c bus %s not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    data->dev = dev;
    data->current_poll_ms = cfg->low_poll_ms;
    data->last_move_time = k_uptime_get_32();
    data->last_poll_time = data->last_move_time;

    if (!tp_wq_started) {
        struct k_work_queue_config wq_cfg = {
            .name = "hanc_tp_wq",
            .no_yield = false,
        };
        k_work_queue_start(&tp_wq, tp_wq_stack,
                           K_THREAD_STACK_SIZEOF(tp_wq_stack),
                           TP_WQ_PRIORITY, &wq_cfg);
        tp_wq_started = true;
    }

    k_work_init_delayable(&data->poll_work, tp_poll_work);
    k_work_reschedule_for_queue(&tp_wq, &data->poll_work,
                                K_MSEC(cfg->low_poll_ms));

    LOG_INF("hanc trackpoint ready on %s addr 0x%02x", cfg->bus.bus->name, cfg->bus.addr);
    return 0;
}

#define TP_INIT(n)                                                              \
    static struct tp_data tp_data_##n;                                          \
    static const struct tp_config tp_cfg_##n = {                                \
        .bus                 = I2C_DT_SPEC_INST_GET(n),                         \
        .low_poll_ms         = DT_INST_PROP(n, low_poll_ms),                    \
        .high_poll_ms        = DT_INST_PROP(n, high_poll_ms),                   \
        .idle_timeout_ms     = DT_INST_PROP(n, idle_timeout_ms),                \
        .dead_zone           = DT_INST_PROP(n, dead_zone),                      \
        .offset_filter_shift = DT_INST_PROP(n, offset_filter_shift),            \
        .scale_shift         = DT_INST_PROP(n, scale_shift),                    \
        .max_delta           = DT_INST_PROP(n, max_delta),                      \
        .invert_x            = DT_INST_PROP(n, invert_x),                       \
        .invert_y            = DT_INST_PROP(n, invert_y),                       \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, tp_init, NULL,                                     \
                          &tp_data_##n, &tp_cfg_##n,                            \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TP_INIT)
