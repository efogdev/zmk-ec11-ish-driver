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

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
#include <zmk_runtime_config/runtime_config.h>
#else
#define ZRC_GET(key, default_val) (default_val)
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(EC11_ISH, CONFIG_SENSOR_LOG_LEVEL);

static void setup_a_int(const struct device *dev, bool enable) {
    const struct ec11_ish_config *cfg = dev->config;
    if (gpio_pin_interrupt_configure_dt(&cfg->a, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set A pin GPIO interrupt");
    }
}

static void setup_b_int(const struct device *dev, bool enable) {
    const struct ec11_ish_config *cfg = dev->config;
    if (gpio_pin_interrupt_configure_dt(&cfg->b, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set B pin GPIO interrupt");
    }
}

static void ec11_a_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins) {
    struct ec11_ish_data *drv_data = CONTAINER_OF(cb, struct ec11_ish_data, a_gpio_cb);
    setup_a_int(drv_data->dev, false);
    k_work_reschedule(&drv_data->a_work,
                      K_MSEC(ZRC_GET("ec11/debounce_ms", CONFIG_EC11_ISH_DEBOUNCE_MS)));
}

static void ec11_b_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins) {
    struct ec11_ish_data *drv_data = CONTAINER_OF(cb, struct ec11_ish_data, b_gpio_cb);
    setup_b_int(drv_data->dev, false);
    k_work_reschedule(&drv_data->b_work,
                      K_MSEC(ZRC_GET("ec11/debounce_ms", CONFIG_EC11_ISH_DEBOUNCE_MS)));
}

static void ec11_a_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct ec11_ish_data *drv_data = CONTAINER_OF(dwork, struct ec11_ish_data, a_work);
    const struct device *dev = drv_data->dev;

    if (drv_data->handler) {
        drv_data->handler(dev, drv_data->trigger);
    }

    setup_a_int(dev, true);
}

static void ec11_b_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct ec11_ish_data *drv_data = CONTAINER_OF(dwork, struct ec11_ish_data, b_work);
    const struct device *dev = drv_data->dev;

    if (drv_data->handler) {
        drv_data->handler(dev, drv_data->trigger);
    }

    setup_b_int(dev, true);
}

int ec11_ish_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                         sensor_trigger_handler_t handler) {
    struct ec11_ish_data *drv_data = dev->data;

    setup_a_int(dev, false);
    setup_b_int(dev, false);

    drv_data->trigger = trig;
    drv_data->handler = handler;

    setup_a_int(dev, true);
    setup_b_int(dev, true);

    return 0;
}

int ec11_ish_init_interrupt(const struct device *dev) {
    struct ec11_ish_data *drv_data = dev->data;
    const struct ec11_ish_config *drv_cfg = dev->config;

    drv_data->dev = dev;

    k_work_init_delayable(&drv_data->a_work, ec11_a_work_cb);
    k_work_init_delayable(&drv_data->b_work, ec11_b_work_cb);

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

    return 0;
}
