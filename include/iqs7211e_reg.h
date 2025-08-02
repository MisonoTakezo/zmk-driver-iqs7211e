#pragma once

#include <stdint.h>

#ifndef AZOTEQ_IQS7211E_ADDRESS
#    define AZOTEQ_IQS7211E_ADDRESS (0x56 << 1)
#endif

#ifndef AZOTEQ_IQS7211E_TIMEOUT_MS
#    define AZOTEQ_IQS7211E_TIMEOUT_MS 100
#endif

#ifndef AZOTEQ_IQS7211E_RDY_PIN
#    define AZOTEQ_IQS7211E_RDY_PIN 21
#endif

// Product number
#define AZOTEQ_IQS7211E_PRODUCT_NUM 0x0458

// Register addresses
#define IQS7211E_MM_PROD_NUM 0x00
#define IQS7211E_MM_MAJOR_VERSION_NUM 0x01
#define IQS7211E_MM_MINOR_VERSION_NUM 0x02
#define IQS7211E_MM_RELATIVE_X 0x0A
#define IQS7211E_MM_RELATIVE_Y 0x0B
#define IQS7211E_MM_GESTURE_X 0x0C
#define IQS7211E_MM_GESTURE_Y 0x0D
#define IQS7211E_MM_GESTURES 0x0E
#define IQS7211E_MM_INFO_FLAGS 0x0F
#define IQS7211E_MM_FINGER_1_X 0x10
#define IQS7211E_MM_FINGER_1_Y 0x11
#define IQS7211E_MM_FINGER_2_X 0x14
#define IQS7211E_MM_FINGER_2_Y 0x15
#define IQS7211E_MM_SYS_CONTROL 0x33
#define IQS7211E_MM_CONFIG_SETTINGS 0x34
#define IQS7211E_MM_X_RESOLUTION 0x43
#define IQS7211E_MM_Y_RESOLUTION 0x44

#define IQS7211E_MM_ALP_ATI_COMP_A 0x1F
#define IQS7211E_MM_TP_GLOBAL_MIRRORS 0x21
#define IQS7211E_MM_ACTIVE_MODE_RR 0x28
#define IQS7211E_MM_ALP_SETUP 0x36
#define IQS7211E_MM_TP_TOUCH_SET_CLEAR_THR 0x38
#define IQS7211E_MM_LP1_FILTERS 0x3B
#define IQS7211E_MM_TP_CONV_FREQ 0x3D
#define IQS7211E_MM_TP_RX_SETTINGS 0x41
#define IQS7211E_MM_SETTINGS_VERSION 0x4A
#define IQS7211E_MM_GESTURE_ENABLE 0x4B
#define IQS7211E_MM_RX_TX_MAPPING_0_1 0x56
#define IQS7211E_MM_PROXA_CYCLE0 0x5D
#define IQS7211E_MM_PROXA_CYCLE10 0x6C
#define IQS7211E_MM_PROXA_CYCLE20 0x7B

// Bit definitions
#define IQS7211E_SHOW_RESET_BIT 7
#define IQS7211E_RE_ATI_OCCURRED_BIT 4
#define IQS7211E_ACK_RESET_BIT 7
#define IQS7211E_TP_RE_ATI_BIT 5
#define IQS7211E_ALP_RE_ATI_BIT 6
#define IQS7211E_SW_RESET_BIT 1
#define IQS7211E_EVENT_MODE_BIT 0
#define IQS7211E_TP_MOVEMENT_BIT 2
#define IQS7211E_NUM_FINGERS_BIT_0 1
#define IQS7211E_NUM_FINGERS_BIT_1 2

// Gesture bits
#define IQS7211E_GESTURE_SINGLE_TAP_BIT 0
#define IQS7211E_GESTURE_PRESS_HOLD_BIT 3

// Byte swap macros
#define AZOTEQ_IQS7211E_SWAP_H_L_BYTES(x) (((x & 0xFF) << 8) | ((x & 0xFF00) >> 8))
#define AZOTEQ_IQS7211E_COMBINE_H_L_BYTES(h, l) (((int16_t)(h << 8)) | l)

// Data structures
typedef union {
    struct {
        uint8_t l;
        uint8_t h;
    };
    uint16_t combined;
} azoteq_iqs7211e_report_rate_t;

typedef union {
    struct {
        uint8_t l;
        uint8_t h;
    };
    int16_t combined;
} azoteq_iqs7211e_coordinate_t;

typedef struct {
    azoteq_iqs7211e_coordinate_t relative_x;
    azoteq_iqs7211e_coordinate_t relative_y;
    azoteq_iqs7211e_coordinate_t gesture_x;
    azoteq_iqs7211e_coordinate_t gesture_y;
    uint8_t                      gestures[2];
    uint8_t                      info_flags[2];
    azoteq_iqs7211e_coordinate_t finger_1_x;
    azoteq_iqs7211e_coordinate_t finger_1_y;
    azoteq_iqs7211e_coordinate_t finger_2_x;
    azoteq_iqs7211e_coordinate_t finger_2_y;
} azoteq_iqs7211e_base_data_t;

// Resolution structure
typedef struct {
    uint16_t x_resolution;
    uint16_t y_resolution;
} azoteq_iqs7211e_resolution_t;
