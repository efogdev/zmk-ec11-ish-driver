/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

struct ec11_ish_config {
    const struct gpio_dt_spec a;
    const struct gpio_dt_spec b;

    const uint16_t steps;
};

struct ec11_ish_data {
    uint8_t ab_state;
    int8_t pulses;
    int8_t processed_pulses;
    bool ready_to_report;

    int64_t last_report_time;
    int8_t last_direction;

    struct gpio_callback a_gpio_cb;
    struct gpio_callback b_gpio_cb;
    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

    struct k_work_delayable work;
    struct k_work _work;
};

int ec11_ish_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                     sensor_trigger_handler_t handler);

int ec11_ish_init_interrupt(const struct device *dev);
