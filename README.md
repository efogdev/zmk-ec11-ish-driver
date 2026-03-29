# ZMK EC11-ish Driver

A ZMK sensor driver for Alps EC11-style rotary encoders with aggressive noise mitigation and missed-pulse compensation.

## Why this exists

The standard EC11 driver works fine for clean signals. Real encoders on handwired builds — especially ones physically close to other components or on long flex cables — don't produce clean signals. This driver adds recursive re-sampling, per-line debouncing, and a compensation window that rounds incomplete pulses to the expected count so you don't get random half-detents.

## Device Tree

```dts
&spi0 {
    scroll_encoder: encoder@0 {
        compatible = "efog,ec11-ish";
        a-gpios = <&gpio0 3 (GPIO_ACTIVE_HIGH | GPIO_PULL_UP)>;
        b-gpios = <&gpio0 28 (GPIO_ACTIVE_HIGH | GPIO_PULL_UP)>;
        steps = <20>;
        pulses = <4>;
    };
};
```

- `steps`: total pulses per full mechanical rotation (check your encoder's datasheet)
- `pulses`: pulses per detent — typically 4 for most EC11 variants, 2 for half-resolution modes

Wire it up in your keymap the same way you'd use any ZMK sensor:

```dts
/ {
    sensors {
        compatible = "zmk,keymap-sensors";
        sensors = <&scroll_encoder>;
        triggers-per-rotation = <20>;
    };
};
```

## Configuration

```kconfig
CONFIG_EC11_ISH=y

# Debounce each GPIO line independently (ms). 0 = disabled.
CONFIG_EC11_ISH_DEBOUNCE_MS=2

# How many times to re-sample before giving up on detecting motion.
# Helps on noisy lines where the first read lands between states.
CONFIG_EC11_ISH_MAX_RECURSION_DEPTH=250

# Enable missed-pulse compensation
CONFIG_EC11_ISH_COMPENSATE_MISSES=y

# Only compensate if at least half the expected pulses fired
CONFIG_EC11_ISH_COMPENSATE_MINIMUM_HALF=y

# Window (ms) in which compensation is evaluated
CONFIG_EC11_ISH_TRIGGER_WINDOW=350
```

All of these — except `CONFIG_EC11_ISH` itself — can also be tuned at runtime if `CONFIG_ZMK_RUNTIME_CONFIG` is enabled (see below).

## Runtime tuning

If `zmk-runtime-config` is in your module list:

```
rtcfg set ec11/debounce_ms 3
rtcfg set ec11/rec_depth 100
rtcfg set ec11/trigger_window 200
rtcfg set ec11/do_comp 1
rtcfg set ec11/comp_half 1
```

Changes are persisted to flash immediately. Revert a parameter to its compiled default with `rtcfg reset ec11/debounce_ms`.
