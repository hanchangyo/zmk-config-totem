/*
 * Runtime-tuning API for the hanc,trackpoint-i2c input driver.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

int      hanc_trackpoint_set_sensitivity(const struct device *dev, uint16_t percent);
int      hanc_trackpoint_adjust_sensitivity(const struct device *dev, int16_t delta_percent);
int      hanc_trackpoint_step_sensitivity(const struct device *dev, int direction);
uint16_t hanc_trackpoint_get_sensitivity(const struct device *dev);

int      hanc_trackpoint_set_invert_x(const struct device *dev, bool invert);
int      hanc_trackpoint_set_invert_y(const struct device *dev, bool invert);
int      hanc_trackpoint_toggle_invert_x(const struct device *dev);
int      hanc_trackpoint_toggle_invert_y(const struct device *dev);

int      hanc_trackpoint_set_swap_xy(const struct device *dev, bool swap);
int      hanc_trackpoint_toggle_swap_xy(const struct device *dev);

int      hanc_trackpoint_reset_tuning(const struct device *dev);

#ifdef __cplusplus
}
#endif
