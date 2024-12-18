#define DT_DRV_COMPAT nxp_mpr121

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

#include "input_mpr121.h"

LOG_MODULE_REGISTER(mpr121, CONFIG_INPUT_LOG_LEVEL);

static int mpr121_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct mpr121_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int mpr121_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct mpr121_config *config = dev->config;
    return config->write(dev, addr, val);
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int mpr121_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct mpr121_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, MPR121_READ | addr, buf, len);
}

static int mpr121_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct mpr121_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, MPR121_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int mpr121_spi_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct mpr121_config *config = dev->config;
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = MPR121_READ | addr;
    memset(&tx_buffer[1], MPR121_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = len + 3,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    return ret;
}

static int mpr121_spi_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct mpr121_config *config = dev->config;
    uint8_t tx_buffer[2] = {MPR121_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }

    if (rx_buffer[1] != MPR121_FILLER) {
        LOG_ERR("bad ret val %d - %d", rx_buffer[0], rx_buffer[1]);
        return -EIO;
    }

    k_usleep(50);

    return ret;
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int set_int(const struct device *dev, const bool en) {
    const struct mpr121_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int mpr121_clear_status(const struct device *dev) {
    int ret = mpr121_write(dev, MPR121_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int mpr121_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = mpr121_write(dev, MPR121_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_CONTROL, MPR121_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = mpr121_seq_read(dev, MPR121_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = mpr121_seq_read(dev, MPR121_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = mpr121_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int mpr121_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = mpr121_write(dev, MPR121_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_CONTROL, MPR121_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = mpr121_seq_read(dev, MPR121_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = mpr121_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static void mpr121_report_data(const struct device *dev) {
    const struct mpr121_config *config = dev->config;
    uint8_t packet[3];
    int ret;
    ret = mpr121_seq_read(dev, MPR121_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 1, "MPR121 Status1");

    if (!(packet[0] & MPR121_STATUS1_SW_DR)) {
        return;
    }
    ret = mpr121_seq_read(dev, MPR121_2_2_PACKET0, packet, 3);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 3, "MPR121 Packets");

    struct mpr121_data *data = dev->data;
    uint8_t btn = packet[0] &
                  (MPR121_PACKET0_BTN_PRIM | MPR121_PACKET0_BTN_SEC | MPR121_PACKET0_BTN_AUX);

    int8_t dx = (int8_t)packet[1];
    int8_t dy = (int8_t)packet[2];

    if (packet[0] & MPR121_PACKET0_X_SIGN) {
        WRITE_BIT(dx, 7, 1);
    }
    if (packet[0] & MPR121_PACKET0_Y_SIGN) {
        WRITE_BIT(dy, 7, 1);
    }

    if (data->in_int) {
        LOG_DBG("Clearing status bit");
        ret = mpr121_clear_status(dev);
        data->in_int = true;
    }

    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
            }
        }
    }

    data->btn_cache = btn;

    input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);

    return;
}

static void mpr121_work_cb(struct k_work *work) {
    struct mpr121_data *data = CONTAINER_OF(work, struct mpr121_data, work);
    mpr121_report_data(data->dev);
}

static void mpr121_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct mpr121_data *data = CONTAINER_OF(cb, struct mpr121_data, gpio_cb);
    data->in_int = true;
    k_work_submit(&data->work);
}

static int mpr121_adc_sensitivity_reg_value(enum mpr121_sensitivity sensitivity) {
    switch (sensitivity) {
    case MPR121_SENSITIVITY_1X:
        return MPR121_TRACKING_ADC_CONFIG_1X;
    case MPR121_SENSITIVITY_2X:
        return MPR121_TRACKING_ADC_CONFIG_2X;
    case MPR121_SENSITIVITY_3X:
        return MPR121_TRACKING_ADC_CONFIG_3X;
    case MPR121_SENSITIVITY_4X:
        return MPR121_TRACKING_ADC_CONFIG_4X;
    default:
        return MPR121_TRACKING_ADC_CONFIG_1X;
    }
}

static int mpr121_tune_edge_sensitivity(const struct device *dev) {
    const struct mpr121_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = mpr121_era_read(dev, MPR121_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0) {
        LOG_WRN("Failed to read X val");
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = mpr121_era_read(dev, MPR121_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0) {
        LOG_WRN("Failed to read Y val");
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = mpr121_era_write(dev, MPR121_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = mpr121_era_write(dev, MPR121_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int mpr121_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct mpr121_config *config = dev->config;

    uint8_t val;
    int ret = mpr121_era_read(dev, MPR121_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= mpr121_adc_sensitivity_reg_value(config->sensitivity);

    ret = mpr121_era_write(dev, MPR121_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = mpr121_era_read(dev, MPR121_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int mpr121_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = mpr121_seq_read(dev, MPR121_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = mpr121_write(dev, MPR121_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do {
        mpr121_seq_read(dev, MPR121_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int mpr121_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = mpr121_seq_read(dev, MPR121_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & MPR121_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, MPR121_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = mpr121_write(dev, MPR121_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}

static int mpr121_init(const struct device *dev) {
    struct mpr121_data *data = dev->data;
    const struct mpr121_config *config = dev->config;
    int ret;

    uint8_t fw_id[2];
    ret = mpr121_seq_read(dev, MPR121_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    data->in_int = false;
    k_msleep(10);
    ret = mpr121_write(dev, MPR121_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = mpr121_write(dev, MPR121_SYS_CFG, MPR121_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = mpr121_write(dev, MPR121_Z_IDLE, 0x05); // No Z-Idle packets
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    ret = mpr121_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = mpr121_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }
    ret = mpr121_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    if (config->sleep_en) {
        ret = mpr121_set_sleep(dev, true);
        if (ret < 0) {
            return ret;
        }
    }

    uint8_t packet[1];
    ret = mpr121_seq_read(dev, MPR121_SLEEP_INTERVAL, packet, 1);

    if (ret >= 0) {
        LOG_DBG("Default sleep interval %d", packet[0]);
    }

    ret = mpr121_write(dev, MPR121_SLEEP_INTERVAL, 255);
    if (ret <= 0) {
        LOG_DBG("Failed to update sleep interaval %d", ret);
    }

    uint8_t feed_cfg2 = MPR121_FEED_CFG2_EN_IM | MPR121_FEED_CFG2_EN_BTN_SCRL;
    if (config->no_taps) {
        feed_cfg2 |= MPR121_FEED_CFG2_DIS_TAP;
    }
    if (config->rotate_90) {
        feed_cfg2 |= MPR121_FEED_CFG2_ROTATE_90;
    }
    ret = mpr121_write(dev, MPR121_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 = MPR121_FEED_CFG1_EN_FEED;
    if (config->x_invert) {
        feed_cfg1 |= MPR121_FEED_CFG1_INV_X;
    }

    if (config->y_invert) {
        feed_cfg1 |= MPR121_FEED_CFG1_INV_Y;
    }
    if (feed_cfg1) {
        ret = mpr121_write(dev, MPR121_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    mpr121_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, mpr121_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, mpr121_work_cb);

    mpr121_write(dev, MPR121_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int mpr121_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return set_int(dev, false);
    case PM_DEVICE_ACTION_RESUME:
        return set_int(dev, true);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define MPR121_INST(n)                                                                           \
    static struct mpr121_data mpr121_data_##n;                                                 \
    static const struct mpr121_config mpr121_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = mpr121_i2c_seq_read,   \
                     .write = mpr121_i2c_write),                                                 \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                        \
                                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |    \
                                                             SPI_TRANSFER_MSB | SPI_MODE_CPHA,     \
                                                         0)},                                      \
                     .seq_read = mpr121_spi_seq_read, .write = mpr121_spi_write)),             \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .no_taps = DT_INST_PROP(n, no_taps),                                                       \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, MPR121_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, mpr121_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, mpr121_init, PM_DEVICE_DT_INST_GET(n), &mpr121_data_##n,          \
                          &mpr121_config_##n, POST_KERNEL, CONFIG_INPUT_MPR121_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MPR121_INST)