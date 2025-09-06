/*
 * Copyright 2025 sekigon-gonnoc
 * SPDX-License-Identifier: GPL-2.0 or later
 */

#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>

#include "../include/iqs7211e_reg.h"
#include "../include/iqs7211e_init.h"

LOG_MODULE_REGISTER(iqs7211e, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT azoteq_iqs7211e

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define IQS7211E_TIMEOUT_MS 100
#define IQS7211E_RESET_DELAY_MS 50
#define IQS7211E_ATI_TIMEOUT_CYCLES 600 // 30 seconds at 50ms intervals

struct iqs7211e_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec power_gpio;
};

struct iqs7211e_data {
    const struct device *dev;
    struct k_work motion_work;
    struct k_work_delayable click_work;
    struct gpio_callback motion_cb;
    uint16_t product_number;
    bool init_complete;
    int16_t previous_x;
    int16_t previous_y;
    bool previous_valid;
    // Gesture state tracking
    int64_t last_touch_time;
    int64_t last_tap_time;
    bool is_clicking;
    bool double_tap_hold;
    uint8_t tap_count;
    int16_t tap_start_x, tap_start_y;
    uint8_t pending_click_type; // 0=none, 1=left
    // Pinch gesture tracking
    int16_t finger_2_prev_x, finger_2_prev_y;
    bool finger_2_prev_valid;
    uint16_t initial_distance;
    bool pinch_active;
};

static uint16_t calculate_distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    int32_t dx = x2 - x1;
    int32_t dy = y2 - y1;
    // Simple distance calculation (avoiding sqrt for performance)
    return (uint16_t)(abs(dx) + abs(dy));
}

static int iqs7211e_i2c_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    const struct iqs7211e_config *cfg = dev->config;
    return i2c_burst_read_dt(&cfg->i2c, reg, data, len);
}

static int iqs7211e_i2c_write_reg(const struct device *dev, uint8_t reg, const uint8_t *data, uint8_t len) {
    const struct iqs7211e_config *cfg = dev->config;
    return i2c_burst_write_dt(&cfg->i2c, reg, data, len);
}

static bool iqs7211e_is_ready(const struct device *dev) {
    const struct iqs7211e_config *cfg = dev->config;
    
    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        return true; // Assume ready if no IRQ pin configured
    }
    
    // RDY pin is active LOW, so device is ready when pin is LOW
    return !gpio_pin_get_dt(&cfg->irq_gpio);
}

static void iqs7211e_wait_for_ready(const struct device *dev, uint16_t timeout_ms) {
    uint16_t elapsed = 0;
    
    while (!iqs7211e_is_ready(dev) && elapsed < timeout_ms) {
        k_sleep(K_MSEC(1));
        elapsed++;
    }
    
    if (elapsed >= timeout_ms) {
        LOG_WRN("RDY timeout after %dms", timeout_ms);
    }
}

static int iqs7211e_get_base_data(const struct device *dev, azoteq_iqs7211e_base_data_t *base_data) {
    uint8_t transfer_bytes[8];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 50);
    
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for data read");
        return -EIO;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_INFO_FLAGS, transfer_bytes, 8);
    if (ret < 0) {
        return ret;
    }
    
    base_data->info_flags[0] = transfer_bytes[0];
    base_data->info_flags[1] = transfer_bytes[1];
    base_data->finger_1_x.l = transfer_bytes[2];
    base_data->finger_1_x.h = transfer_bytes[3];
    base_data->finger_1_y.l = transfer_bytes[4];
    base_data->finger_1_y.h = transfer_bytes[5];
    base_data->finger_2_x.l = transfer_bytes[6];
    base_data->finger_2_x.h = transfer_bytes[7];
    
    return 0;
}

static int iqs7211e_reset(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read system control: %d", ret);
        return ret;
    }
    
    transfer_bytes[1] |= (1 << IQS7211E_SW_RESET_BIT);
    
    return iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
}

static int iqs7211e_set_event_mode(const struct device *dev, bool enabled) {
    uint8_t transfer_bytes[2];
    int ret;
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_CONFIG_SETTINGS, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    if (enabled) {
        transfer_bytes[1] |= (1 << IQS7211E_EVENT_MODE_BIT);
    } else {
        transfer_bytes[1] &= ~(1 << IQS7211E_EVENT_MODE_BIT);
    }
    
    return iqs7211e_i2c_write_reg(dev, IQS7211E_MM_CONFIG_SETTINGS, transfer_bytes, 2);
}

static int iqs7211e_acknowledge_reset(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 50);
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    iqs7211e_wait_for_ready(dev, 50);
    transfer_bytes[0] |= (1 << IQS7211E_ACK_RESET_BIT);
    
    ret = iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    LOG_DBG("Acknowledged reset, status %d", ret);
    
    return ret;
}

static int iqs7211e_reati(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 100);
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] |= (1 << IQS7211E_TP_RE_ATI_BIT);
    
    ret = iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    LOG_DBG("RE-ATI enabled, status %d", ret);
    
    return ret;
}

static uint16_t iqs7211e_get_product(const struct device *dev) {
    struct iqs7211e_data *data = dev->data;
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 100);
    
    if (!iqs7211e_is_ready(dev)) {
        data->product_number = 0xff;
        LOG_WRN("Device not ready for product read");
        return 0;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_PROD_NUM, transfer_bytes, 2);
    if (ret == 0) {
        data->product_number = transfer_bytes[0] | (transfer_bytes[1] << 8);
    }
    
    LOG_DBG("Product number %u, status %d", data->product_number, ret);
    return data->product_number;
}

static int iqs7211e_write_memory_map(const struct device *dev) {
    uint8_t transfer_bytes[30];
    int ret = 0;
    
    LOG_DBG("Writing memory map");
    
    // 1. Write ALP Compensation (0x1F - 0x20)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = ALP_COMPENSATION_A_0;
    transfer_bytes[1] = ALP_COMPENSATION_A_1;
    transfer_bytes[2] = ALP_COMPENSATION_B_0;
    transfer_bytes[3] = ALP_COMPENSATION_B_1;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_ALP_ATI_COMP_A, transfer_bytes, 4);
    LOG_DBG("\t1. Write ALP Compensation");
    
    // 2. Write ATI Settings (0x21 - 0x27)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
    transfer_bytes[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
    transfer_bytes[2] = TP_COMPENSATION_DIV;
    transfer_bytes[3] = TP_REF_DRIFT_LIMIT;
    transfer_bytes[4] = TP_ATI_TARGET_0;
    transfer_bytes[5] = TP_ATI_TARGET_1;
    transfer_bytes[6] = TP_MIN_COUNT_REATI_0;
    transfer_bytes[7] = TP_MIN_COUNT_REATI_1;
    transfer_bytes[8] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
    transfer_bytes[9] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
    transfer_bytes[10] = ALP_COMPENSATION_DIV;
    transfer_bytes[11] = ALP_LTA_DRIFT_LIMIT;
    transfer_bytes[12] = ALP_ATI_TARGET_0;
    transfer_bytes[13] = ALP_ATI_TARGET_1;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_TP_GLOBAL_MIRRORS, transfer_bytes, 14);
    LOG_DBG("\t2. Write ATI Settings");
    
    // 3. Write Report rates and timings (0x28 - 0x32)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0]  = ACTIVE_MODE_REPORT_RATE_0;
    transfer_bytes[1]  = ACTIVE_MODE_REPORT_RATE_1;
    transfer_bytes[2]  = IDLE_TOUCH_MODE_REPORT_RATE_0;
    transfer_bytes[3]  = IDLE_TOUCH_MODE_REPORT_RATE_1;
    transfer_bytes[4]  = IDLE_MODE_REPORT_RATE_0;
    transfer_bytes[5]  = IDLE_MODE_REPORT_RATE_1;
    transfer_bytes[6]  = LP1_MODE_REPORT_RATE_0;
    transfer_bytes[7]  = LP1_MODE_REPORT_RATE_1;
    transfer_bytes[8]  = LP2_MODE_REPORT_RATE_0;
    transfer_bytes[9]  = LP2_MODE_REPORT_RATE_1;
    transfer_bytes[10] = ACTIVE_MODE_TIMEOUT_0;
    transfer_bytes[11] = ACTIVE_MODE_TIMEOUT_1;
    transfer_bytes[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
    transfer_bytes[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
    transfer_bytes[14] = IDLE_MODE_TIMEOUT_0;
    transfer_bytes[15] = IDLE_MODE_TIMEOUT_1;
    transfer_bytes[16] = LP1_MODE_TIMEOUT_0;
    transfer_bytes[17] = LP1_MODE_TIMEOUT_1;
    transfer_bytes[18] = REATI_RETRY_TIME;
    transfer_bytes[19] = REF_UPDATE_TIME;
    transfer_bytes[20] = I2C_TIMEOUT_0;
    transfer_bytes[21] = I2C_TIMEOUT_1;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_ACTIVE_MODE_RR, transfer_bytes, 22);
    LOG_DBG("\t3. Write Report rates and timings");

    // 4. Write System control settings (0x33 - 0x35)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = SYSTEM_CONTROL_0;
    transfer_bytes[1] = SYSTEM_CONTROL_1;
    transfer_bytes[2] = CONFIG_SETTINGS0;
    transfer_bytes[3] = CONFIG_SETTINGS1;
    transfer_bytes[4] = OTHER_SETTINGS_0;
    transfer_bytes[5] = OTHER_SETTINGS_1;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 6);
    LOG_DBG("\t4. Write System control settings");

    // 5. Write ALP Settings (0x36 - 0x37)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = ALP_SETUP_0;
    transfer_bytes[1] = ALP_SETUP_1;
    transfer_bytes[2] = ALP_TX_ENABLE_0;
    transfer_bytes[3] = ALP_TX_ENABLE_1;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_ALP_SETUP, transfer_bytes, 4);
    LOG_DBG("\t5. Write ALP Settings");

    // 6. Write Threshold settings (0x38 - 0x3A)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = TRACKPAD_TOUCH_SET_THRESHOLD;
    transfer_bytes[1] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
    transfer_bytes[2] = ALP_THRESHOLD_0;
    transfer_bytes[3] = ALP_THRESHOLD_1;
    transfer_bytes[4] = ALP_SET_DEBOUNCE;
    transfer_bytes[5] = ALP_CLEAR_DEBOUNCE;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_TP_TOUCH_SET_CLEAR_THR, transfer_bytes, 6);
    LOG_DBG("\t6. Write Threshold settings");

    // 7. Write Filter Betas (0x3B - 0x3C)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = ALP_COUNT_BETA_LP1;
    transfer_bytes[1] = ALP_LTA_BETA_LP1;
    transfer_bytes[2] = ALP_COUNT_BETA_LP2;
    transfer_bytes[3] = ALP_LTA_BETA_LP2;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_LP1_FILTERS, transfer_bytes, 4);
    LOG_DBG("\t7. Write Filter Betas");

    // 8. Write Hardware settings (0x3D - 0x40)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
    transfer_bytes[1] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
    transfer_bytes[2] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
    transfer_bytes[3] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
    transfer_bytes[4] = TRACKPAD_HARDWARE_SETTINGS_0;
    transfer_bytes[5] = TRACKPAD_HARDWARE_SETTINGS_1;
    transfer_bytes[6] = ALP_HARDWARE_SETTINGS_0;
    transfer_bytes[7] = ALP_HARDWARE_SETTINGS_1;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_TP_CONV_FREQ, transfer_bytes, 8);
    LOG_DBG("\t8. Write Hardware settings");

    // 9. Write TP Settings (0x41 - 0x49)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0]  = TRACKPAD_SETTINGS_0_0;
    transfer_bytes[1]  = TRACKPAD_SETTINGS_0_1;
    transfer_bytes[2]  = TRACKPAD_SETTINGS_1_0;
    transfer_bytes[3]  = TRACKPAD_SETTINGS_1_1;
    transfer_bytes[4]  = X_RESOLUTION_0;
    transfer_bytes[5]  = X_RESOLUTION_1;
    transfer_bytes[6]  = Y_RESOLUTION_0;
    transfer_bytes[7]  = Y_RESOLUTION_1;
    transfer_bytes[8]  = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
    transfer_bytes[9]  = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
    transfer_bytes[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
    transfer_bytes[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
    transfer_bytes[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
    transfer_bytes[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
    transfer_bytes[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
    transfer_bytes[15] = FINGER_SPLIT_FACTOR;
    transfer_bytes[16] = X_TRIM_VALUE;
    transfer_bytes[17] = Y_TRIM_VALUE;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_TP_RX_SETTINGS, transfer_bytes, 18);
    LOG_DBG("\t9. Write TP Settings");

    // 10. Write Version numbers (0x4A)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = MINOR_VERSION;
    transfer_bytes[1] = MAJOR_VERSION;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_SETTINGS_VERSION, transfer_bytes, 2);
    LOG_DBG("\t10. Write Version numbers");

    // 11. Write Gesture Settings (0x4B - 0x55)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0]  = GESTURE_ENABLE_0;
    transfer_bytes[1]  = GESTURE_ENABLE_1;
    transfer_bytes[2]  = TAP_TOUCH_TIME_0;
    transfer_bytes[3]  = TAP_TOUCH_TIME_1;
    transfer_bytes[4]  = TAP_WAIT_TIME_0;
    transfer_bytes[5]  = TAP_WAIT_TIME_1;
    transfer_bytes[6]  = TAP_DISTANCE_0;
    transfer_bytes[7]  = TAP_DISTANCE_1;
    transfer_bytes[8]  = HOLD_TIME_0;
    transfer_bytes[9]  = HOLD_TIME_1;
    transfer_bytes[10] = SWIPE_TIME_0;
    transfer_bytes[11] = SWIPE_TIME_1;
    transfer_bytes[12] = SWIPE_X_DISTANCE_0;
    transfer_bytes[13] = SWIPE_X_DISTANCE_1;
    transfer_bytes[14] = SWIPE_Y_DISTANCE_0;
    transfer_bytes[15] = SWIPE_Y_DISTANCE_1;
    transfer_bytes[16] = SWIPE_X_CONS_DIST_0;
    transfer_bytes[17] = SWIPE_X_CONS_DIST_1;
    transfer_bytes[18] = SWIPE_Y_CONS_DIST_0;
    transfer_bytes[19] = SWIPE_Y_CONS_DIST_1;
    transfer_bytes[20] = SWIPE_ANGLE;
    transfer_bytes[21] = PALM_THRESHOLD;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_GESTURE_ENABLE, transfer_bytes, 22);
    LOG_DBG("\t11. Write Gesture Settings");

    // 12. Write Rx Tx Map Settings (0x56 - 0x5C)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0]  = RX_TX_MAP_0;
    transfer_bytes[1]  = RX_TX_MAP_1;
    transfer_bytes[2]  = RX_TX_MAP_2;
    transfer_bytes[3]  = RX_TX_MAP_3;
    transfer_bytes[4]  = RX_TX_MAP_4;
    transfer_bytes[5]  = RX_TX_MAP_5;
    transfer_bytes[6]  = RX_TX_MAP_6;
    transfer_bytes[7]  = RX_TX_MAP_7;
    transfer_bytes[8]  = RX_TX_MAP_8;
    transfer_bytes[9]  = RX_TX_MAP_9;
    transfer_bytes[10] = RX_TX_MAP_10;
    transfer_bytes[11] = RX_TX_MAP_11;
    transfer_bytes[12] = RX_TX_MAP_12;
    transfer_bytes[13] = RX_TX_MAP_FILLER;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_RX_TX_MAPPING_0_1, transfer_bytes, 14);
    LOG_DBG("\t12. Write Rx Tx Map Settings");

    // 13. Write Cycle 0 - 9 Settings (0x5D - 0x6B)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0]  = PLACEHOLDER_0;
    transfer_bytes[1]  = CH_1_CYCLE_0;
    transfer_bytes[2]  = CH_2_CYCLE_0;
    transfer_bytes[3]  = PLACEHOLDER_1;
    transfer_bytes[4]  = CH_1_CYCLE_1;
    transfer_bytes[5]  = CH_2_CYCLE_1;
    transfer_bytes[6]  = PLACEHOLDER_2;
    transfer_bytes[7]  = CH_1_CYCLE_2;
    transfer_bytes[8]  = CH_2_CYCLE_2;
    transfer_bytes[9]  = PLACEHOLDER_3;
    transfer_bytes[10] = CH_1_CYCLE_3;
    transfer_bytes[11] = CH_2_CYCLE_3;
    transfer_bytes[12] = PLACEHOLDER_4;
    transfer_bytes[13] = CH_1_CYCLE_4;
    transfer_bytes[14] = CH_2_CYCLE_4;
    transfer_bytes[15] = PLACEHOLDER_5;
    transfer_bytes[16] = CH_1_CYCLE_5;
    transfer_bytes[17] = CH_2_CYCLE_5;
    transfer_bytes[18] = PLACEHOLDER_6;
    transfer_bytes[19] = CH_1_CYCLE_6;
    transfer_bytes[20] = CH_2_CYCLE_6;
    transfer_bytes[21] = PLACEHOLDER_7;
    transfer_bytes[22] = CH_1_CYCLE_7;
    transfer_bytes[23] = CH_2_CYCLE_7;
    transfer_bytes[24] = PLACEHOLDER_8;
    transfer_bytes[25] = CH_1_CYCLE_8;
    transfer_bytes[26] = CH_2_CYCLE_8;
    transfer_bytes[27] = PLACEHOLDER_9;
    transfer_bytes[28] = CH_1_CYCLE_9;
    transfer_bytes[29] = CH_2_CYCLE_9;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_PROXA_CYCLE0, transfer_bytes, 30);
    LOG_DBG("\t13. Write Cycle 0 - 9 Settings");

    // 14. Write Cycle 10 - 19 Settings (0x6C - 0x7A)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0]  = PLACEHOLDER_10;
    transfer_bytes[1]  = CH_1_CYCLE_10;
    transfer_bytes[2]  = CH_2_CYCLE_10;
    transfer_bytes[3]  = PLACEHOLDER_11;
    transfer_bytes[4]  = CH_1_CYCLE_11;
    transfer_bytes[5]  = CH_2_CYCLE_11;
    transfer_bytes[6]  = PLACEHOLDER_12;
    transfer_bytes[7]  = CH_1_CYCLE_12;
    transfer_bytes[8]  = CH_2_CYCLE_12;
    transfer_bytes[9]  = PLACEHOLDER_13;
    transfer_bytes[10] = CH_1_CYCLE_13;
    transfer_bytes[11] = CH_2_CYCLE_13;
    transfer_bytes[12] = PLACEHOLDER_14;
    transfer_bytes[13] = CH_1_CYCLE_14;
    transfer_bytes[14] = CH_2_CYCLE_14;
    transfer_bytes[15] = PLACEHOLDER_15;
    transfer_bytes[16] = CH_1_CYCLE_15;
    transfer_bytes[17] = CH_2_CYCLE_15;
    transfer_bytes[18] = PLACEHOLDER_16;
    transfer_bytes[19] = CH_1_CYCLE_16;
    transfer_bytes[20] = CH_2_CYCLE_16;
    transfer_bytes[21] = PLACEHOLDER_17;
    transfer_bytes[22] = CH_1_CYCLE_17;
    transfer_bytes[23] = CH_2_CYCLE_17;
    transfer_bytes[24] = PLACEHOLDER_18;
    transfer_bytes[25] = CH_1_CYCLE_18;
    transfer_bytes[26] = CH_2_CYCLE_18;
    transfer_bytes[27] = PLACEHOLDER_19;
    transfer_bytes[28] = CH_1_CYCLE_19;
    transfer_bytes[29] = CH_2_CYCLE_19;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_PROXA_CYCLE10, transfer_bytes, 30);
    LOG_DBG("\t14. Write Cycle 10 - 19 Settings");

    // 15. Write Cycle 20 Settings (0x7B - 0x7C)
    iqs7211e_wait_for_ready(dev, 100);
    transfer_bytes[0] = PLACEHOLDER_20;
    transfer_bytes[1] = CH_1_CYCLE_20;
    transfer_bytes[2] = CH_2_CYCLE_20;
    ret |= iqs7211e_i2c_write_reg(dev, IQS7211E_MM_PROXA_CYCLE20, transfer_bytes, 3);
    LOG_DBG("\t15. Write Cycle 20 Settings");
    
    LOG_DBG("Memory map write complete, status: %d", ret);
    return ret;
}

static int iqs7211e_check_reset(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 50);
    
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for reset check");
        return -EIO;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_INFO_FLAGS, transfer_bytes, 2);
    if (ret < 0) {
        return ret;
    }
    
    return (transfer_bytes[0] & (1 << IQS7211E_SHOW_RESET_BIT)) ? 0 : -EAGAIN;
}

static bool iqs7211e_read_ati_active(const struct device *dev) {
    uint8_t transfer_bytes[2];
    int ret;
    
    iqs7211e_wait_for_ready(dev, 500);
    
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for ATI check");
        return true;
    }
    
    ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_SYS_CONTROL, transfer_bytes, 2);
    if (ret < 0) {
        return true;
    }
    
    LOG_DBG("ATI active check, flags: 0x%02X", transfer_bytes[0]);
    return (transfer_bytes[0] & (1 << IQS7211E_TP_RE_ATI_BIT)) != 0;
}

static void iqs7211e_click_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(dwork, struct iqs7211e_data, click_work);
    const struct device *dev = data->dev;
    
    if (data->pending_click_type == 1) {
        // Left click release
        LOG_DBG("Single tap - release");
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
    }
    data->pending_click_type = 0;
}

static void iqs7211e_motion_work_handler(struct k_work *work) {
    struct iqs7211e_data *data = CONTAINER_OF(work, struct iqs7211e_data, motion_work);
    const struct device *dev = data->dev;
    azoteq_iqs7211e_base_data_t base_data = {0};
    int ret;
    int64_t current_time = k_uptime_get();
    
    LOG_DBG("Motion work handler started");
    if (!data->init_complete) {
        LOG_WRN("Device not initialized, skipping motion handling");
        return;
    }
    
    // Only read data if device is ready
    if (!iqs7211e_is_ready(dev)) {
        LOG_WRN("Device not ready for motion data");
        return;
    }
    
    ret = iqs7211e_get_base_data(dev, &base_data);
    if (ret < 0) {
        LOG_WRN("Get report failed, status: %d", ret);
        return;
    }
 
    uint8_t finger_count = base_data.info_flags[1] & 0x03;
    
    if (finger_count == 1) {
        // Single finger handling
        uint16_t finger_1_x = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_x.h, base_data.finger_1_x.l);
        uint16_t finger_1_y = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_y.h, base_data.finger_1_y.l);
        
        // Reset pinch state when going to single finger
        if (data->pinch_active) {
            data->pinch_active = false;
            LOG_DBG("Pinch gesture ended");
        }
        
        if (!data->previous_valid || data->finger_2_prev_valid) {
            // Touch start or transition from two finger
            data->tap_start_x = finger_1_x;
            data->tap_start_y = finger_1_y;
            data->last_touch_time = current_time;
        } else {
            // Normal finger movement
            int16_t x = finger_1_x - data->previous_x;
            int16_t y = finger_1_y - data->previous_y;
            
            LOG_DBG("Movement: x=%4d y=%4d", x, y);
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        }
        
        data->previous_x = finger_1_x;
        data->previous_y = finger_1_y;
        data->previous_valid = true;
        data->finger_2_prev_valid = false;
        
    } else if (finger_count == 2) {
        // Two finger handling - pinch zoom
        uint16_t finger_1_x = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_x.h, base_data.finger_1_x.l);
        uint16_t finger_1_y = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_1_y.h, base_data.finger_1_y.l);
        uint16_t finger_2_x = AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(base_data.finger_2_x.h, base_data.finger_2_x.l);
        
        // Read finger 2 Y coordinate
        uint8_t finger_2_y_bytes[2];
        ret = iqs7211e_i2c_read_reg(dev, IQS7211E_MM_FINGER_2_Y, finger_2_y_bytes, 2);
        uint16_t finger_2_y = (ret == 0) ? AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(finger_2_y_bytes[1], finger_2_y_bytes[0]) : 0;
        
        uint16_t current_distance = calculate_distance(finger_1_x, finger_1_y, finger_2_x, finger_2_y);
        
        if (!data->finger_2_prev_valid) {
            // Two finger touch start
            data->initial_distance = current_distance;
            data->pinch_active = true;
            data->last_touch_time = current_time;
            LOG_DBG("Pinch gesture started, initial distance: %d", data->initial_distance);
        } else if (data->pinch_active) {
            // Pinch gesture in progress
            int16_t distance_change = current_distance - data->initial_distance;
            
            // Only trigger zoom if distance change is significant
            if (abs(distance_change) > 30) {
                if (distance_change > 0) {
                    // Zoom in (Command + scroll up)
                    LOG_DBG("Zoom in, distance change: %d", distance_change);
                    input_report_key(dev, INPUT_KEY_LEFTMETA, 1, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL, 1, false, K_FOREVER);
                    input_report_key(dev, INPUT_KEY_LEFTMETA, 0, true, K_FOREVER);
                } else {
                    // Zoom out (Command + scroll down)
                    LOG_DBG("Zoom out, distance change: %d", distance_change);
                    input_report_key(dev, INPUT_KEY_LEFTMETA, 1, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL, -1, false, K_FOREVER);
                    input_report_key(dev, INPUT_KEY_LEFTMETA, 0, true, K_FOREVER);
                }
                // Update reference distance to prevent repeated events
                data->initial_distance = current_distance;
            }
        }
        
        data->previous_x = finger_1_x;
        data->previous_y = finger_1_y;
        data->finger_2_prev_x = finger_2_x;
        data->finger_2_prev_y = finger_2_y;
        data->previous_valid = true;
        data->finger_2_prev_valid = true;
        
    } else {
        // No fingers - handle touch end events
        if (data->pinch_active) {
            data->pinch_active = false;
            LOG_DBG("Pinch gesture ended");
        }
        
        if (data->previous_valid && !data->finger_2_prev_valid) {
            // Single finger tap handling
            int64_t touch_duration = current_time - data->last_touch_time;
            int16_t tap_distance = abs(data->previous_x - data->tap_start_x) + abs(data->previous_y - data->tap_start_y);
            
            if (touch_duration < 200 && tap_distance < 50) { // Quick tap with minimal movement
                int64_t tap_interval = current_time - data->last_tap_time;
                
                if (tap_interval < 400 && data->tap_count == 1) { // Double tap
                    LOG_DBG("Double tap - start hold click");
                    data->double_tap_hold = true;
                    data->is_clicking = true;
                    input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
                    data->tap_count = 0;
                } else {
                    // Single tap
                    if (!data->double_tap_hold) {
                        LOG_DBG("Single tap - press");
                        input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
                        data->pending_click_type = 1;
                        k_work_schedule(&data->click_work, K_MSEC(50));
                    }
                    data->tap_count = 1;
                }
                data->last_tap_time = current_time;
            } else if (data->double_tap_hold && data->is_clicking) {
                // Release double-tap hold
                LOG_DBG("Release double tap hold");
                data->double_tap_hold = false;
                data->is_clicking = false;
                input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
            }
            
            // Reset tap count if too much time passed
            if (current_time - data->last_tap_time > 600) {
                data->tap_count = 0;
            }
        }
        
        data->previous_valid = false;
        data->finger_2_prev_valid = false;
    }
}

static void iqs7211e_motion_handler(const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins) {
    struct iqs7211e_data *data = CONTAINER_OF(cb, struct iqs7211e_data, motion_cb);
    k_work_submit(&data->motion_work);
}

static int iqs7211e_configure(const struct device *dev) {
    struct iqs7211e_data *data = dev->data;
    int ret;
    
    LOG_DBG("Initialization started");
    
    // Wait for device to be ready
    iqs7211e_wait_for_ready(dev, 100);
    
    // Software reset
    ret = iqs7211e_reset(dev);
    if (ret < 0) {
        LOG_ERR("Reset failed: %d", ret);
        return ret;
    }
    
    k_sleep(K_MSEC(IQS7211E_RESET_DELAY_MS));
    
    // Wait for device to be ready after reset
    iqs7211e_wait_for_ready(dev, 200);
    
    // Check product number
    if (iqs7211e_get_product(dev) == AZOTEQ_IQS7211E_PRODUCT_NUM) {
        LOG_DBG("Device found");
        
        // Check if reset occurred
        if (iqs7211e_check_reset(dev) == 0) {
            LOG_DBG("Reset event confirmed");
            
            // Write all settings from init file
            ret = iqs7211e_write_memory_map(dev);
            if (ret == 0) {
                // Acknowledge reset
                ret = iqs7211e_acknowledge_reset(dev);
                if (ret < 0) {
                    return ret;
                }
                
                k_sleep(K_MSEC(100));
                
                // Run ATI
                ret = iqs7211e_reati(dev);
                if (ret < 0) {
                    return ret;
                }
                
                // Wait for ATI to complete
                int ati_timeout = IQS7211E_ATI_TIMEOUT_CYCLES;
                while (iqs7211e_read_ati_active(dev) && ati_timeout > 0) {
                    k_sleep(K_MSEC(50));
                    ati_timeout--;
                }
                
                if (ati_timeout > 0) {
                    LOG_DBG("ATI completed");
                    
                    // Wait for device to be ready before setting event mode
                    iqs7211e_wait_for_ready(dev, 500);
                    
                    // Set event mode
                    ret = iqs7211e_set_event_mode(dev, true);
                    if (ret < 0) {
                        return ret;
                    }
                    
                    k_sleep(K_MSEC(ACTIVE_MODE_REPORT_RATE_0 + 1));
                    
                    data->init_complete = true;
                    LOG_DBG("Init complete");
                } else {
                    LOG_ERR("ATI timeout");
                    return -ETIMEDOUT;
                }
            } else {
                LOG_ERR("Memory map write failed");
                return ret;
            }
        } else {
            LOG_ERR("No reset event detected");
            return -EIO;
        }
    } else {
        LOG_ERR("Device not found, product: 0x%04x", data->product_number);
        return -ENODEV;
    }
    
    return 0;
}

static int iqs7211e_init(const struct device *dev) {
    const struct iqs7211e_config *cfg = dev->config;
    struct iqs7211e_data *data = dev->data;
    int ret;
    
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready", cfg->i2c.bus->name);
        return -ENODEV;
    }
    
    data->dev = dev;
    data->init_complete = false;
    data->previous_valid = false;
    data->pending_click_type = 0;
    data->finger_2_prev_valid = false;
    data->pinch_active = false;
    data->initial_distance = 0;
    
    k_work_init(&data->motion_work, iqs7211e_motion_work_handler);
    k_work_init_delayable(&data->click_work, iqs7211e_click_work_handler);
    
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    if (gpio_is_ready_dt(&cfg->power_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Power pin configuration failed: %d", ret);
            return ret;
        }
        
        k_sleep(K_MSEC(500));
        
        ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
        if (ret != 0) {
            LOG_ERR("Power pin set failed: %d", ret);
            return ret;
        }
        
        k_sleep(K_MSEC(10));
    }
#endif
    
    if (gpio_is_ready_dt(&cfg->irq_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("IRQ pin configuration failed: %d", ret);
            return ret;
        }
        
        gpio_init_callback(&data->motion_cb, iqs7211e_motion_handler, BIT(cfg->irq_gpio.pin));
        
        ret = gpio_add_callback_dt(&cfg->irq_gpio, &data->motion_cb);
        if (ret < 0) {
            LOG_ERR("Could not set motion callback: %d", ret);
            return ret;
        }

        LOG_DBG("IRQ pin configured, pin: %d", cfg->irq_gpio.pin);
    }
    
    ret = iqs7211e_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }
    
    if (gpio_is_ready_dt(&cfg->irq_gpio)) {
        ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_FALLING);
        if (ret != 0) {
            LOG_ERR("Motion interrupt configuration failed: %d", ret);
            return ret;
        }
    }
    
    ret = pm_device_runtime_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable runtime power management: %d", ret);
        return ret;
    }
    
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int iqs7211e_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct iqs7211e_config *cfg = dev->config;
    struct iqs7211e_data *data = dev->data;
    int ret;
    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        if (gpio_is_ready_dt(&cfg->irq_gpio)) {
            ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
            if (ret < 0) {
                LOG_ERR("Failed to disable IRQ interrupt: %d", ret);
                return ret;
            }
            
            ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_DISCONNECTED);
            if (ret < 0) {
                LOG_ERR("Failed to disconnect IRQ GPIO: %d", ret);
                return ret;
            }
        }
        
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        if (gpio_is_ready_dt(&cfg->power_gpio)) {
            ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_DISCONNECTED);
            if (ret < 0) {
                LOG_ERR("Failed to disconnect power: %d", ret);
                return ret;
            }
        }
#endif
        data->init_complete = false;
        break;
        
    case PM_DEVICE_ACTION_RESUME:
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        if (gpio_is_ready_dt(&cfg->power_gpio)) {
            ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_ACTIVE);
            if (ret < 0) {
                LOG_ERR("Failed to enable power: %d", ret);
                return ret;
            }
            k_sleep(K_MSEC(10));
        }
#endif
        
        if (gpio_is_ready_dt(&cfg->irq_gpio)) {
            ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
            if (ret < 0) {
                LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
                return ret;
            }
            
            ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_FALLING);
            if (ret < 0) {
                LOG_ERR("Failed to enable IRQ interrupt: %d", ret);
                return ret;
            }
        }
        
        ret = iqs7211e_configure(dev);
        if (ret < 0) {
            LOG_ERR("Failed to reconfigure device: %d", ret);
            return ret;
        }
        break;
        
    default:
        return -ENOTSUP;
    }
    
    return 0;
}
#endif

#define IQS7211E_INIT(n)                                                                           \
    static const struct iqs7211e_config iqs7211e_cfg_##n = {                                      \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                           \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                          \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                              \
    };                                                                                             \
                                                                                                   \
    static struct iqs7211e_data iqs7211e_data_##n;                                                \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, iqs7211e_pm_action);                                              \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, iqs7211e_init, PM_DEVICE_DT_INST_GET(n), &iqs7211e_data_##n,        \
                          &iqs7211e_cfg_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS7211E_INIT)

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)