# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources(ec11_ish.c)
zephyr_library_sources(ec11_ish_trigger.c)
