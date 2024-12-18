#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

#define MPR121_READ 0xA0
#define MPR121_WRITE 0x80

#define MPR121_AUTOINC 0xFC
#define MPR121_FILLER 0xFB

// Registers
#define MPR121_FW_ID 0x00   // ASIC ID.
#define MPR121_FW_VER 0x01  // Firmware Version Firmware revision number.
#define MPR121_STATUS1 0x02 // Contains status flags about the state of MPR121.
#define MPR121_STATUS1_SW_DR BIT(2)
#define MPR121_STATUS1_SW_CC BIT(3)
#define MPR121_SYS_CFG 0x03 // Contains system operation and configuration bits.
#define MPR121_SYS_CFG_EN_SLEEP_BIT 2
#define MPR121_SYS_CFG_EN_SLEEP BIT(2)
#define MPR121_SYS_CFG_SHUTDOWN BIT(1)
#define MPR121_SYS_CFG_RESET BIT(0)

#define MPR121_FEED_CFG1 0x04 // Contains feed operation and configuration bits.
#define MPR121_FEED_CFG1_EN_FEED BIT(0)
#define MPR121_FEED_CFG1_ABS_MODE BIT(1)
#define MPR121_FEED_CFG1_DIS_FILT BIT(2)
#define MPR121_FEED_CFG1_DIS_X BIT(3)
#define MPR121_FEED_CFG1_DIS_Y BIT(4)
#define MPR121_FEED_CFG1_INV_X BIT(6)
#define MPR121_FEED_CFG1_INV_Y BIT(7)
#define MPR121_FEED_CFG2 0x05               // Contains feed operation and configuration bits.
#define MPR121_FEED_CFG2_EN_IM BIT(0)       // Intellimouse
#define MPR121_FEED_CFG2_DIS_TAP BIT(1)     // Disable all taps
#define MPR121_FEED_CFG2_DIS_SEC BIT(2)     // Disable secondary tap
#define MPR121_FEED_CFG2_DIS_SCRL BIT(3)    // Disable scroll
#define MPR121_FEED_CFG2_DIS_GE BIT(4)      // Disable GlideExtend
#define MPR121_FEED_CFG2_EN_BTN_SCRL BIT(6) // Enable Button Scroll
#define MPR121_FEED_CFG2_ROTATE_90 BIT(7)   // Swap X & Y
#define MPR121_CAL_CFG 0x07                 // Contains calibration configuration bits.
#define MPR121_PS2_AUX 0x08                 // Contains Data register for PS/2 Aux Control.
#define MPR121_SAMPLE 0x09                  // Sample Rate Number of samples generated per second.
#define MPR121_Z_IDLE 0x0A         // Number of Z=0 packets sent when Z goes from >0 to 0.
#define MPR121_Z_SCALER 0x0B       // Contains the pen Z_On threshold.
#define MPR121_SLEEP_INTERVAL 0x0C // Sleep Interval
#define MPR121_SLEEP_TIMER 0x0D    // Sleep Timer
#define MPR121_AG_PACKET0 0x10     // trackpad Data (MPR121 AG)
#define MPR121_2_2_PACKET0 0x12    // trackpad Data
#define MPR121_REG_COUNT 0x18

#define MPR121_REG_ERA_VALUE 0x1B
#define MPR121_REG_ERA_HIGH_BYTE 0x1C
#define MPR121_REG_ERA_LOW_BYTE 0x1D
#define MPR121_REG_ERA_CONTROL 0x1E

#define MPR121_ERA_CONTROL_READ 0x01
#define MPR121_ERA_CONTROL_WRITE 0x02

#define MPR121_ERA_REG_X_AXIS_WIDE_Z_MIN 0x0149
#define MPR121_ERA_REG_Y_AXIS_WIDE_Z_MIN 0x0168
#define MPR121_ERA_REG_TRACKING_ADC_CONFIG 0x0187

#define MPR121_TRACKING_ADC_CONFIG_1X 0x00
#define MPR121_TRACKING_ADC_CONFIG_2X 0x40
#define MPR121_TRACKING_ADC_CONFIG_3X 0x80
#define MPR121_TRACKING_ADC_CONFIG_4X 0xC0

#define MPR121_PACKET0_BTN_PRIM BIT(0) // Primary button
#define MPR121_PACKET0_BTN_SEC BIT(1)  // Secondary button
#define MPR121_PACKET0_BTN_AUX BIT(2)  // Auxiliary (middle?) button
#define MPR121_PACKET0_X_SIGN BIT(4)   // X delta sign
#define MPR121_PACKET0_Y_SIGN BIT(5)   // Y delta sign

struct mpr121_data {
    uint8_t btn_cache;
    bool in_int;
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
};

enum mpr121_sensitivity {
    MPR121_SENSITIVITY_1X,
    MPR121_SENSITIVITY_2X,
    MPR121_SENSITIVITY_3X,
    MPR121_SENSITIVITY_4X,
};

typedef int (*mpr121_seq_read_t)(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                   const uint8_t len);
typedef int (*mpr121_write_t)(const struct device *dev, const uint8_t addr, const uint8_t val);

struct mpr121_config {
    union {
        struct i2c_dt_spec i2c;
        struct spi_dt_spec spi;
    } bus;

    mpr121_seq_read_t seq_read;
    mpr121_write_t write;

    bool rotate_90, sleep_en, no_taps, x_invert, y_invert;
    enum mpr121_sensitivity sensitivity;
    uint8_t x_axis_z_min, y_axis_z_min;
    const struct gpio_dt_spec dr;
};

int mpr121_set_sleep(const struct device *dev, bool enabled);