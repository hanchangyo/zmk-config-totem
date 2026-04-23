/*
 * zmk,behavior-tp-tune
 *
 * Keymap-bindable behavior that tunes the hanc,trackpoint-i2c device at
 * runtime. Binding params:
 *   param1 = action code (see include/dt-bindings/zmk/tp_tune.h)
 *   param2 = optional value (e.g. percent for TP_SENS_SET)
 */

#define DT_DRV_COMPAT zmk_behavior_tp_tune

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include <dt-bindings/zmk/tp_tune.h>
#include <hanc/trackpoint.h>

LOG_MODULE_REGISTER(behavior_tp_tune, CONFIG_ZMK_LOG_LEVEL);

struct behavior_tp_tune_config {
	const struct device *tp_dev;
};

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
				     struct zmk_behavior_binding_event event)
{
	const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);

	if (dev == NULL) {
		return ZMK_BEHAVIOR_OPAQUE;
	}

	const struct behavior_tp_tune_config *cfg = dev->config;
	const struct device *tp = cfg->tp_dev;

	if (tp == NULL || !device_is_ready(tp)) {
		LOG_WRN("tp_tune target device not ready");
		return ZMK_BEHAVIOR_OPAQUE;
	}

	switch (binding->param1) {
	case TP_SENS_UP:
		hanc_trackpoint_step_sensitivity(tp, +1);
		break;
	case TP_SENS_DN:
		hanc_trackpoint_step_sensitivity(tp, -1);
		break;
	case TP_SENS_SET:
		hanc_trackpoint_set_sensitivity(tp, (uint16_t)binding->param2);
		break;
	case TP_INV_X:
		hanc_trackpoint_toggle_invert_x(tp);
		break;
	case TP_INV_Y:
		hanc_trackpoint_toggle_invert_y(tp);
		break;
	case TP_SWAP:
		hanc_trackpoint_toggle_swap_xy(tp);
		break;
	case TP_RESET:
		hanc_trackpoint_reset_tuning(tp);
		break;
	default:
		LOG_WRN("unknown tp_tune action %u", binding->param1);
		break;
	}

	return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
				      struct zmk_behavior_binding_event event)
{
	ARG_UNUSED(binding);
	ARG_UNUSED(event);
	return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_tp_tune_driver_api = {
	.binding_pressed = on_keymap_binding_pressed,
	.binding_released = on_keymap_binding_released,
};

static int behavior_tp_tune_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define KP_INST(n)                                                                             \
	static const struct behavior_tp_tune_config tp_tune_cfg_##n = {                        \
		.tp_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, device)),                           \
	};                                                                                     \
	BEHAVIOR_DT_INST_DEFINE(n, behavior_tp_tune_init, NULL, NULL, &tp_tune_cfg_##n,        \
				POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,              \
				&behavior_tp_tune_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)
