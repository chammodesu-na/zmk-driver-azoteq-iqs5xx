/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>
#include "iqs5xx.h"

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_read_burst(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};

    return i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, len);
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {IQS5XX_END_COMM_WINDOW >> 8, IQS5XX_END_COMM_WINDOW & 0xFF, 0x00};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static int iqs5xx_setup_device(const struct device *dev);

static void iqs5xx_button_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

    for (int i = 0; i < 3; i++) {
        if (data->buttons_pressed & BIT(i)) {
            LOG_DBG("Releasing synthetic button %d", i);
            input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
            data->buttons_pressed &= ~BIT(i);
        }
    }
}

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    uint8_t base_data[IQS5XX_BASE_DATA_LEN];
    int ret;

    // The status registers are contiguous, so a single burst read gives us a
    // coherent snapshot of the cycle. Reading them one at a time both costs
    // several I2C transactions per report and can straddle two report cycles.
    ret = iqs5xx_read_burst(dev, IQS5XX_BASE_DATA, base_data, sizeof(base_data));
    if (ret < 0) {
        LOG_ERR("Failed to read base data: %d", ret);
        goto end_comm;
    }

    uint8_t gesture_events_0 = base_data[IQS5XX_BD_GESTURE_EVENTS_0];
    uint8_t gesture_events_1 = base_data[IQS5XX_BD_GESTURE_EVENTS_1];
    uint8_t sys_info_0 = base_data[IQS5XX_BD_SYSTEM_INFO_0];
    uint8_t sys_info_1 = base_data[IQS5XX_BD_SYSTEM_INFO_1];
    uint8_t num_fingers = base_data[IQS5XX_BD_NUM_FINGERS];
    int16_t rel_x = (int16_t)((base_data[IQS5XX_BD_REL_X] << 8) | base_data[IQS5XX_BD_REL_X + 1]);
    int16_t rel_y = (int16_t)((base_data[IQS5XX_BD_REL_Y] << 8) | base_data[IQS5XX_BD_REL_Y + 1]);

    if (sys_info_0 & IQS5XX_SHOW_RESET) {
        // A reset (the watchdog, or a brown-out) drops the device back to its
        // defaults, so acknowledging it is not enough: everything configured at
        // init has to be written again or the trackpad silently comes back with
        // stock gestures, axes and report rates.
        LOG_INF("Device reset detected, reapplying configuration");
        iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);

        if (data->touching) {
            data->touching = false;
            if (config->report_touch_state) {
                input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
            }
        }

        // iqs5xx_setup_device() ends the communication window itself.
        iqs5xx_setup_device(dev);
        return;
    }

    // Finger presence is reported before the gesture branches below, all of
    // which can jump straight to end_comm. Doing it here means the layer a
    // touch activates goes down on the very first cycle the finger lands and
    // comes back up on the first cycle after it lifts.
    bool touching = num_fingers > 0;
    if (config->report_touch_state && touching != data->touching) {
        data->touching = touching;
        input_report_key(dev, INPUT_BTN_TOUCH, touching ? 1 : 0, true, K_FOREVER);
    }

    bool tp_movement = (sys_info_1 & IQS5XX_TP_MOVEMENT) != 0;
    bool scroll = (gesture_events_1 & IQS5XX_SCROLL) != 0;
    if (!scroll) {
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    }

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events_0 & IQS5XX_SINGLE_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_0;
    } else if (gesture_events_1 & IQS5XX_TWO_FINGER_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_1;
    }

    bool hold_became_active = (gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released = !(gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && data->active_hold;

    if (hold_became_active) {
        LOG_INF("Hold became active");
        input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
        data->active_hold = true;
    } else if (hold_released) {
        LOG_INF("Hold became inactive");
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
        data->active_hold = false;
    } else if (button_pressed) {
        k_work_cancel_delayable(&data->button_release_work);
        input_report_key(dev, button_code, 1, true, K_FOREVER);
        data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);
        k_work_schedule(&data->button_release_work, K_MSEC(100));
    } else if (scroll) {
        int16_t scroll_div = 32;

        if (rel_x != 0) {
            if (!config->natural_scroll_x) {
                rel_x *= -1;
            }
            data->scroll_x_acc += rel_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                K_FOREVER);
                data->scroll_x_acc %= scroll_div;
            }
            goto end_comm;
        }
        if (rel_y != 0) {
            if (config->natural_scroll_y) {
                rel_y *= -1;
            }
            data->scroll_y_acc += rel_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_y_acc %= scroll_div;
            }
            goto end_comm;
        }
    } else if (tp_movement) {
        if (rel_x != 0 || rel_y != 0) {
            input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
        }
    }

end_comm:
    iqs5xx_end_comm_window(dev);
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    k_work_submit(&data->work);
}

// Report rate and dwell time of each power mode. The device steps down
// Active -> Idle-Touch / Idle -> LP1 -> LP2 on its own; each step down slows
// the scan rate, which is what makes the trackpad feel asleep when you come
// back to it. Writing IQS5XX_TIMEOUT_DISABLED to a timeout stops the device
// from ever taking that step.
static int iqs5xx_setup_power_modes(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret;

    const struct {
        uint16_t reg;
        uint16_t val;
    } report_rates[] = {
        {IQS5XX_ACTIVE_REPORT_RATE, config->active_report_rate},
        {IQS5XX_IDLE_TOUCH_REPORT_RATE, config->idle_touch_report_rate},
        {IQS5XX_IDLE_REPORT_RATE, config->idle_report_rate},
        {IQS5XX_LP1_REPORT_RATE, config->lp1_report_rate},
        {IQS5XX_LP2_REPORT_RATE, config->lp2_report_rate},
    };

    for (size_t i = 0; i < ARRAY_SIZE(report_rates); i++) {
        ret = iqs5xx_write_reg16(dev, report_rates[i].reg, report_rates[i].val);
        if (ret < 0) {
            LOG_ERR("Failed to set report rate at 0x%04x: %d", report_rates[i].reg, ret);
            return ret;
        }
    }

    const struct {
        uint16_t reg;
        uint8_t val;
    } timeouts[] = {
        {IQS5XX_ACTIVE_MODE_TIMEOUT, config->active_mode_timeout},
        {IQS5XX_IDLE_TOUCH_TIMEOUT, config->idle_touch_timeout},
        {IQS5XX_IDLE_TIMEOUT, config->idle_timeout},
        {IQS5XX_LP1_TIMEOUT, config->lp1_timeout},
    };

    for (size_t i = 0; i < ARRAY_SIZE(timeouts); i++) {
        ret = iqs5xx_write_reg8(dev, timeouts[i].reg, timeouts[i].val);
        if (ret < 0) {
            LOG_ERR("Failed to set timeout at 0x%04x: %d", timeouts[i].reg, ret);
            return ret;
        }
    }

    return 0;
}

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret;

    uint8_t event_mask = IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT;
    // Without TOUCH_EVENT the device only opens a communication window while a
    // finger is moving, so a finger that lands and rests, or one that lifts
    // without moving first, is never reported.
    if (config->report_touch_state) {
        event_mask |= IQS5XX_TOUCH_EVENT;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1, event_mask);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }

    ret = iqs5xx_setup_power_modes(dev);
    if (ret < 0) {
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_BOTTOM_BETA, config->bottom_beta);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom beta: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_STATIONARY_THRESH, config->stationary_threshold);
    if (ret < 0) {
        LOG_ERR("Failed to set bottom stationary threshold: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_FILTER_SETTINGS,
                            IQS5XX_IIR_FILTER | IQS5XX_MAV_FILTER | IQS5XX_ALP_COUNT_FILTER);
    if (ret < 0) {
        LOG_ERR("Failed to configure filter settings: %d", ret);
        return ret;
    }

    uint8_t single_finger_gestures = 0;
    single_finger_gestures |= config->one_finger_tap ? IQS5XX_SINGLE_TAP : 0;
    single_finger_gestures |= config->press_and_hold ? IQS5XX_PRESS_AND_HOLD : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_SINGLE_FINGER_GESTURES_CONF, single_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure single finger gestures: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg16(dev, IQS5XX_HOLD_TIME, config->press_and_hold_time);
    if (ret < 0) {
        LOG_ERR("Failed to configure the hold time: %d", ret);
        return ret;
    }

    uint8_t two_finger_gestures = 0;
    two_finger_gestures |= config->two_finger_tap ? IQS5XX_TWO_FINGER_TAP : 0;
    two_finger_gestures |= config->scroll ? IQS5XX_SCROLL : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, two_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure multi finger gestures: %d", ret);
        return ret;
    }

    uint8_t xy_config = 0;
    xy_config |= config->flip_x ? IQS5XX_FLIP_X : 0;
    xy_config |= config->flip_y ? IQS5XX_FLIP_Y : 0;
    xy_config |= config->switch_xy ? IQS5XX_SWITCH_XY_AXIS : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure axes: %d", ret);
        return ret;
    }

    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, IQS5XX_SETUP_COMPLETE | IQS5XX_WDT);
    if (ret < 0) {
        LOG_ERR("Failed to configure system: %d", ret);
        return ret;
    }

    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        LOG_ERR("Failed to end comm window during initialization: %d", ret);
        return ret;
    }

    return 0;
}

static int iqs5xx_pm_action(const struct device *dev, enum pm_device_action action)
{
    const struct iqs5xx_config *config = dev->config;

    if (!config->reset_gpio.port) {
        return -ENOTSUP;
    }

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND: {
        gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_DISABLE);
        gpio_pin_set_dt(&config->reset_gpio, 1);

        // The device is about to stop reporting, so release the touch state
        // rather than leaving a layer held down across the suspend.
        struct iqs5xx_data *data = dev->data;
        if (data->touching) {
            data->touching = false;
            if (config->report_touch_state) {
                input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
            }
        }
        break;
    }

    case PM_DEVICE_ACTION_RESUME:
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(100);
        iqs5xx_setup_device(dev);
        gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

/*
 * PM notifier: System OFF(딥슬립) 진입 직전 호출됨.
 * nRF52840은 System OFF 중 GPIO 출력 레벨을 latch로 유지하므로,
 * 여기서 reset 핀을 assert(물리적 LOW)하면 칩이 리셋 상태로 고정됨.
 * 웨이크업은 완전 재부팅이므로 iqs5xx_init()이 자동 재실행 → resume 불필요.
 *
 * 주의: state_entry는 스케줄러 잠금 상태에서 호출됨.
 *       GPIO 레지스터 쓰기만 가능, k_msleep() 등 블로킹 호출 금지.
 */
static void iqs5xx_pm_state_entry(enum pm_state state)
{
    if (state != PM_STATE_SOFT_OFF) {
        return;
    }

    const struct device *dev = DEVICE_DT_INST_GET(0);
    const struct iqs5xx_config *config = dev->config;

    if (!config->reset_gpio.port) {
        return;
    }

    gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_DISABLE);
    gpio_pin_set_dt(&config->reset_gpio, 1); /* 물리적 LOW → IQS5XX 리셋 고정 */
}

static struct pm_notifier iqs5xx_pm_notifier = {
    .state_entry = iqs5xx_pm_state_entry,
    .state_exit  = NULL,
};

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);

    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(1);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(10);
    }

    if (!gpio_is_ready_dt(&config->rdy_gpio)) {
        LOG_ERR("RDY GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY GPIO: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
    ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add RDY callback: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY interrupt: %d", ret);
        return ret;
    }

    k_msleep(100);

    ret = iqs5xx_setup_device(dev);
    if (ret < 0) {
        LOG_ERR("Failed to setup device: %d", ret);
        return ret;
    }

    pm_notifier_register(&iqs5xx_pm_notifier);

    data->initialized = true;
    LOG_INF("IQS5xx trackpad initialized");

    return 0;
}

#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                         \
        .press_and_hold = DT_INST_PROP(n, press_and_hold),                                         \
        .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                         \
        .scroll = DT_INST_PROP(n, scroll),                                                         \
        .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                                     \
        .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                                     \
        .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),                       \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
        .active_report_rate = DT_INST_PROP(n, active_report_rate_ms),                               \
        .idle_touch_report_rate = DT_INST_PROP(n, idle_touch_report_rate_ms),                       \
        .idle_report_rate = DT_INST_PROP(n, idle_report_rate_ms),                                   \
        .lp1_report_rate = DT_INST_PROP(n, lp1_report_rate_ms),                                     \
        .lp2_report_rate = DT_INST_PROP(n, lp2_report_rate_ms),                                     \
        .active_mode_timeout = DT_INST_PROP(n, active_mode_timeout_s),                              \
        .idle_touch_timeout = DT_INST_PROP(n, idle_touch_timeout_s),                                \
        .idle_timeout = DT_INST_PROP(n, idle_timeout_s),                                            \
        .lp1_timeout = DT_INST_PROP(n, lp1_timeout_s),                                              \
        .report_touch_state = DT_INST_PROP(n, report_touch_state),                                  \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, iqs5xx_pm_action);                                                \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, PM_DEVICE_DT_INST_GET(n), &iqs5xx_data_##n,             \
                          &iqs5xx_config_##n, POST_KERNEL,                                         \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)
