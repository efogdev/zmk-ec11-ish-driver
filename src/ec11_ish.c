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

LOG_MODULE_REGISTER(EC11_ISH, CONFIG_SENSOR_LOG_LEVEL);

static void ec11_work_handler(struct k_work *work);

static int ec11_get_ab_state(const struct device *dev) {
    const struct ec11_ish_config *drv_cfg = dev->config;
    return (gpio_pin_get_dt(&drv_cfg->a) << 1) | gpio_pin_get_dt(&drv_cfg->b);
}

static void process_pulses_and_trigger(const struct device *dev) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;
    const int64_t now = k_uptime_get();
    int8_t direction = 0;
    if (drv_data->pulses > 0) {
        direction = 1;
    } else if (drv_data->pulses < 0) {
        direction = -1;
    }

    if (direction != 0 &&
        direction == drv_data->last_direction &&
        (now - drv_data->last_report_time) < CONFIG_EC11_ISH_DEBOUNCE_MS) {
        LOG_INF("Debouncing: ignoring report in same direction within %d ms",
                CONFIG_EC11_ISH_DEBOUNCE_MS);
        drv_data->pulses = 0;
        return;
    }

    drv_data->processed_pulses = direction;
    drv_data->ready_to_report = true;
    drv_data->last_report_time = now;
    drv_data->last_direction = direction;

    if (drv_data->handler) {
        drv_data->handler(dev, drv_data->trigger);
    }

    drv_data->pulses = 0;
}

static void ec11_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    const struct ec11_ish_data *drv_data = CONTAINER_OF(dwork, struct ec11_ish_data, work);
    const struct device *dev = drv_data->dev;

    if (drv_data->pulses != 0) {
        process_pulses_and_trigger(dev);
    }
}

static int ec11_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;
    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION);

    int8_t delta;
    const uint8_t val = ec11_get_ab_state(dev);

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

    if (delta != 0) {
        drv_data->pulses += delta;

        if (drv_data->pulses == delta) {
            k_work_schedule(&drv_data->work, K_MSEC(CONFIG_EC11_ISH_TRIGGER_WINDOW));
        }

        if (abs(drv_data->pulses) >= drv_cfg->steps) {
            k_work_cancel_delayable(&drv_data->work);
            process_pulses_and_trigger(dev);
        }
    }

    drv_data->ab_state = val;
    return 0;
}

static int ec11_channel_get(const struct device *dev, const enum sensor_channel chan,
                            struct sensor_value *val) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;

    if (chan != SENSOR_CHAN_ROTATION) {
        return -ENOTSUP;
    }

    if (drv_data->ready_to_report) {
        val->val1 = drv_data->processed_pulses * (360 / drv_cfg->steps);
        val->val2 = 0;
        drv_data->ready_to_report = false;
    } else {
        val->val1 = 0;
        val->val2 = 0;
    }

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
    k_work_init_delayable(&drv_data->work, ec11_work_handler);

    if (ec11_ish_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }

    drv_data->ab_state = ec11_get_ab_state(dev);
    drv_data->pulses = 0;
    drv_data->processed_pulses = 0;
    drv_data->ready_to_report = false;
    drv_data->last_report_time = 0;
    drv_data->last_direction = 0;

    return 0;
}

#define EC11_INST(n)                                                                               \
    static struct ec11_ish_data ec11_ish_data_##n;                                                 \
    static const struct ec11_ish_config ec11_cfg_##n = {                                           \
        .a = GPIO_DT_SPEC_INST_GET(n, a_gpios),                                                    \
        .b = GPIO_DT_SPEC_INST_GET(n, b_gpios),                                                    \
        .steps = DT_INST_PROP_OR(n, steps, 1),                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, ec11_ish_init, NULL, &ec11_ish_data_##n, &ec11_cfg_##n, POST_KERNEL,  \
                          CONFIG_SENSOR_INIT_PRIORITY, &ec11_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EC11_INST)
