/*
 * Copyright (c) 2026 batao9
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_motion_threshold

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/input_processor.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/keymap.h>

LOG_MODULE_REGISTER(zmk_motion_threshold, CONFIG_ZMK_LOG_LEVEL);

struct motion_threshold_config {
    const struct device *delegate;
    uint16_t threshold;
    uint16_t activation_window_ms;
    uint16_t require_prior_idle_ms;
};

struct motion_threshold_data {
    struct k_mutex lock;
    int32_t accumulated_x;
    int32_t accumulated_y;
    int64_t last_motion_timestamp;
    int64_t last_key_timestamp;
    bool armed;
};

static void reset_motion(struct motion_threshold_data *data) {
    data->accumulated_x = 0;
    data->accumulated_y = 0;
    data->last_motion_timestamp = 0;
    data->armed = false;
}

static bool is_xy_event(const struct input_event *event) {
    return event->type == INPUT_EV_REL &&
           (event->code == INPUT_REL_X || event->code == INPUT_REL_Y);
}

static bool threshold_reached(const struct motion_threshold_config *config,
                              const struct motion_threshold_data *data) {
    int64_t x = data->accumulated_x;
    int64_t y = data->accumulated_y;
    int64_t threshold = config->threshold;

    return (x * x) + (y * y) >= threshold * threshold;
}

static int motion_threshold_handle_event(const struct device *dev, struct input_event *event,
                                         uint32_t param1, uint32_t param2,
                                         struct zmk_input_processor_state *state) {
    if (!is_xy_event(event) || event->value == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const struct motion_threshold_config *config = dev->config;
    struct motion_threshold_data *data = dev->data;
    int64_t now = k_uptime_get();
    bool should_delegate = false;

    int ret = k_mutex_lock(&data->lock, K_FOREVER);
    if (ret < 0) {
        return ret;
    }

    if (config->require_prior_idle_ms > 0 &&
        data->last_key_timestamp + config->require_prior_idle_ms > now) {
        reset_motion(data);
        ret = k_mutex_unlock(&data->lock);
        return ret < 0 ? ret : ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->last_motion_timestamp > 0 &&
        now - data->last_motion_timestamp > config->activation_window_ms) {
        reset_motion(data);
    }

    data->last_motion_timestamp = now;
    /* Keep signed displacement so small back-and-forth sensor jitter cancels out. */
    if (event->code == INPUT_REL_X) {
        data->accumulated_x += event->value;
    } else {
        data->accumulated_y += event->value;
    }

    if (!data->armed && threshold_reached(config, data)) {
        data->armed = true;
        LOG_DBG("Motion threshold reached: x=%d y=%d", data->accumulated_x, data->accumulated_y);
    }

    should_delegate = data->armed || zmk_keymap_layer_active(param1);

    ret = k_mutex_unlock(&data->lock);
    if (ret < 0) {
        return ret;
    }

    if (!should_delegate) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* The delegate controls only the layer; the original pointer event remains unchanged. */
    return zmk_input_processor_handle_event(config->delegate, event, param1, param2, state);
}

static int motion_threshold_init(const struct device *dev) {
    const struct motion_threshold_config *config = dev->config;
    struct motion_threshold_data *data = dev->data;

    if (!device_is_ready(config->delegate)) {
        LOG_ERR("Delegate input processor is not ready");
        return -ENODEV;
    }

    k_mutex_init(&data->lock);
    return 0;
}

static int handle_keycode_state_changed(const zmk_event_t *event) {
    const struct zmk_keycode_state_changed *key_event = as_zmk_keycode_state_changed(event);

    if (!key_event->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

#define RESET_FOR_KEY_EVENT(n)                                                                     \
    {                                                                                              \
        struct motion_threshold_data *data = DEVICE_DT_INST_GET(n)->data;                          \
        int ret = k_mutex_lock(&data->lock, K_FOREVER);                                            \
        if (ret < 0) {                                                                             \
            return ret;                                                                            \
        }                                                                                          \
        data->last_key_timestamp = key_event->timestamp;                                           \
        reset_motion(data);                                                                        \
        ret = k_mutex_unlock(&data->lock);                                                         \
        if (ret < 0) {                                                                             \
            return ret;                                                                            \
        }                                                                                          \
    }

    DT_INST_FOREACH_STATUS_OKAY(RESET_FOR_KEY_EVENT)

#undef RESET_FOR_KEY_EVENT

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(motion_threshold, handle_keycode_state_changed);
ZMK_SUBSCRIPTION(motion_threshold, zmk_keycode_state_changed);

static const struct zmk_input_processor_driver_api motion_threshold_driver_api = {
    .handle_event = motion_threshold_handle_event,
};

#define MOTION_THRESHOLD_INST(n)                                                                   \
    BUILD_ASSERT(DT_INST_PROP(n, threshold) > 0, "Motion threshold must be greater than zero");    \
    BUILD_ASSERT(DT_INST_PROP(n, activation_window_ms) > 0,                                        \
                 "Activation window must be greater than zero");                                   \
    static struct motion_threshold_data motion_threshold_data_##n = {};                            \
    static const struct motion_threshold_config motion_threshold_config_##n = {                    \
        .delegate = DEVICE_DT_GET(DT_INST_PHANDLE(n, delegate)),                                   \
        .threshold = DT_INST_PROP(n, threshold),                                                   \
        .activation_window_ms = DT_INST_PROP(n, activation_window_ms),                             \
        .require_prior_idle_ms = DT_INST_PROP(n, require_prior_idle_ms),                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, motion_threshold_init, NULL, &motion_threshold_data_##n,              \
                          &motion_threshold_config_##n, POST_KERNEL,                               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &motion_threshold_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOTION_THRESHOLD_INST)
