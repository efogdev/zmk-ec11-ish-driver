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

#include "ec11_ish.h"

extern struct ec11_ish_data ec11_driver;

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(EC11_ISH, CONFIG_SENSOR_LOG_LEVEL);

static void setup_int(const struct device *dev, const bool enable) {
    const struct ec11_ish_config *cfg = dev->config;
    LOG_DBG("enabled %s", (enable ? "true" : "false"));

    if (gpio_pin_interrupt_configure_dt(&cfg->a, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set A pin GPIO interrupt");
    }

    if (gpio_pin_interrupt_configure_dt(&cfg->b, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set A pin GPIO interrupt");
    }
}

static void ec11_a_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct ec11_ish_data *drv_data = CONTAINER_OF(cb, struct ec11_ish_data, a_gpio_cb);
    setup_int(drv_data->dev, false);
    k_work_submit(&drv_data->_work);
}

static void ec11_b_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct ec11_ish_data *drv_data = CONTAINER_OF(cb, struct ec11_ish_data, b_gpio_cb);
    setup_int(drv_data->dev, false);
    k_work_submit(&drv_data->_work);
}

static void ec11_thread_cb(const struct device *dev) {
    const struct ec11_ish_data *drv_data = dev->data;
    drv_data->handler(dev, drv_data->trigger);
    setup_int(dev, true);
}

static void ec11_work_cb(struct k_work *work) {
    const struct ec11_ish_data *drv_data = CONTAINER_OF(work, struct ec11_ish_data, _work);
    ec11_thread_cb(drv_data->dev);
}

int ec11_ish_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                     const sensor_trigger_handler_t handler) {
    struct ec11_ish_data *drv_data = dev->data;

    setup_int(dev, false);
    k_msleep(5);

    drv_data->trigger = trig;
    drv_data->handler = handler;

    setup_int(dev, true);
    return 0;
}

int ec11_ish_init_interrupt(const struct device *dev) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;

    drv_data->dev = dev;

    gpio_init_callback(&drv_data->a_gpio_cb, ec11_a_gpio_callback, BIT(drv_cfg->a.pin));
    if (gpio_add_callback(drv_cfg->a.port, &drv_data->a_gpio_cb) < 0) {
        LOG_DBG("Failed to set A callback!");
        return -EIO;
    }

    gpio_init_callback(&drv_data->b_gpio_cb, ec11_b_gpio_callback, BIT(drv_cfg->b.pin));
    if (gpio_add_callback(drv_cfg->b.port, &drv_data->b_gpio_cb) < 0) {
        LOG_DBG("Failed to set B callback!");
        return -EIO;
    }

    k_work_init(&drv_data->_work, ec11_work_cb);
    return 0;
}
