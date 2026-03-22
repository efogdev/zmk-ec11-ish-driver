/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT efog_ec11_ish

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include "ec11_ish.h"

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
#include <zmk_runtime_config/runtime_config.h>
#endif

LOG_MODULE_REGISTER(EC11_ISH, CONFIG_SENSOR_LOG_LEVEL);

static int ec11_sample_fetch_impl(const struct device *dev, const enum sensor_channel chan, const uint16_t depth) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;
    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION);

    const uint32_t now = k_uptime_get_32();
    const uint8_t val = (gpio_pin_get_dt(&drv_cfg->a) << 1) | gpio_pin_get_dt(&drv_cfg->b);
    int8_t delta;

    if (drv_data->last_correct_delta != 0 && depth == 0
        && now - drv_data->last_sample_time > ZRC_GET("ec11/delta_ttl", CONFIG_EC11_ISH_LAST_DELTA_TTL)) {
        drv_data->last_correct_delta = 0;
    }

    switch (val | (drv_data->ab_state << 2)) {
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
        delta = -1;
        break;
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        delta = 1;
        break;
    default:
        delta = 0;
        break;
    }
    
    if (delta == 0 && depth < ZRC_GET("ec11/rec_depth", CONFIG_EC11_ISH_MAX_RECURSION_DEPTH)) {
        return ec11_sample_fetch_impl(dev, chan, depth + 1);
    }

    if (delta == 0 && drv_data->last_correct_delta != 0) {
        return drv_data->last_correct_delta;
    }

    drv_data->last_sample_time = now;
    drv_data->last_correct_delta = delta;
    drv_data->delta = delta;
    drv_data->ab_state = val;
    return 0;
}

static int ec11_sample_fetch(const struct device *dev, const enum sensor_channel chan) {
    return ec11_sample_fetch_impl(dev, chan, 0);
}

static int ec11_channel_get(const struct device *dev, const enum sensor_channel chan,
                            struct sensor_value *val) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    const int32_t numerator = 360 * drv_data->delta;
    val->val1 = numerator / (int32_t)drv_cfg->steps;
    val->val2 = (numerator % (int32_t)drv_cfg->steps) * 1000000 / (int32_t)drv_cfg->steps;
    drv_data->delta = 0;

    return 0;
}

static const struct sensor_driver_api ec11_driver_api = {
    .trigger_set = ec11_ish_trigger_set,
    .sample_fetch = ec11_sample_fetch,
    .channel_get = ec11_channel_get,
};

int ec11_ish_init(const struct device *dev) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;

    LOG_DBG("A: %s %d B: %s %d ", drv_cfg->a.port->name, drv_cfg->a.pin,
            drv_cfg->b.port->name, drv_cfg->b.pin);

    if (!device_is_ready(drv_cfg->a.port)) {
        LOG_ERR("A GPIO device is not ready");
        return -EINVAL;
    }

    if (!device_is_ready(drv_cfg->b.port)) {
        LOG_ERR("B GPIO device is not ready");
        return -EINVAL;
    }

    if (gpio_pin_configure_dt(&drv_cfg->a, GPIO_INPUT)) {
        LOG_DBG("Failed to configure A pin");
        return -EIO;
    }

    if (gpio_pin_configure_dt(&drv_cfg->b, GPIO_INPUT)) {
        LOG_DBG("Failed to configure B pin");
        return -EIO;
    }

    drv_data->dev = dev;

    if (ec11_ish_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }

    drv_data->ab_state = (gpio_pin_get_dt(&drv_cfg->a) << 1) | gpio_pin_get_dt(&drv_cfg->b);
    drv_data->delta = 0;
    drv_data->last_correct_delta = 0;
    drv_data->last_sample_time = 0;
    return 0;
}

#define EC11_INST(n)                                                                               \
    static struct ec11_ish_data ec11_ish_data_##n = {0};                                           \
    static const struct ec11_ish_config ec11_cfg_##n = {                                           \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                    \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                    \
        .steps = DT_INST_PROP_OR(n, steps, 1),                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, ec11_ish_init, NULL, &ec11_ish_data_##n, &ec11_cfg_##n, POST_KERNEL,  \
                          CONFIG_SENSOR_INIT_PRIORITY, &ec11_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EC11_INST)

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
static int ec11_ish_register_runtime_params(void) {
    zrc_register("ec11/debounce_ms", CONFIG_EC11_ISH_DEBOUNCE_MS, 0, 100);
    zrc_register("ec11/rec_depth", CONFIG_EC11_ISH_MAX_RECURSION_DEPTH, 1, 65535);
    zrc_register("ec11/delta_ttl", CONFIG_EC11_ISH_LAST_DELTA_TTL, 1, 65535);
    return 0;
}
SYS_INIT(ec11_ish_register_runtime_params, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif /* CONFIG_ZMK_RUNTIME_CONFIG */
