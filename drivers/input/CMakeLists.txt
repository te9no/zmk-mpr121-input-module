# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_MPR121 input_mpr121.c)

target_sources_ifdef(CONFIG_ZMK_INPUT_MPR121_IDLE_SLEEPER app PRIVATE zmk_mpr121_idle_sleeper.c)