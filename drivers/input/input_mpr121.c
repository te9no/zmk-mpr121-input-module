#define DT_DRV_COMPAT nxp_mpr121

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

#include "input_mpr121.h"

LOG_MODULE_REGISTER(mpr121, CONFIG_INPUT_LOG_LEVEL);

// uncomment to use autoconfig !
// #define AUTOCONFIG // use autoconfig (Yes it works pretty well!)

static int mpr121_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                           const uint8_t len)
{
    const struct mpr121_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int mpr121_write(const struct device *dev, const uint8_t addr, const uint8_t val)
{
    const struct mpr121_config *config = dev->config;
    return config->write(dev, addr, val);
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int mpr121_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                               const uint8_t len)
{
    const struct mpr121_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, MPR121_I2C_ADDRESS | addr, buf, len);
}

static int mpr121_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val)
{
    const struct mpr121_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, MPR121_I2C_ADDRESS | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

/*
static int set_int(const struct device *dev, const bool en)
{
    const struct mpr121_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0)
    {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int mpr121_clear_status(const struct device *dev)
{
    int ret = mpr121_write(dev, MPR121_STATUS1, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int mpr121_era_read(const struct device *dev, const uint16_t addr, uint8_t *val)
{
    int ret;

    set_int(dev, false);

    ret = mpr121_write(dev, MPR121_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_CONTROL, MPR121_ERA_CONTROL_READ);
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do
    {

        ret = mpr121_seq_read(dev, MPR121_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0)
        {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = mpr121_seq_read(dev, MPR121_REG_ERA_VALUE, val, 1);

    if (ret < 0)
    {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = mpr121_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int mpr121_era_write(const struct device *dev, const uint16_t addr, uint8_t val)
{
    int ret;

    set_int(dev, false);

    ret = mpr121_write(dev, MPR121_REG_ERA_VALUE, val);
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = mpr121_write(dev, MPR121_REG_ERA_CONTROL, MPR121_ERA_CONTROL_WRITE);
    if (ret < 0)
    {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do
    {

        ret = mpr121_seq_read(dev, MPR121_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0)
        {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = mpr121_clear_status(dev);

    set_int(dev, true);

    return ret;
}
*/

/**
 *  @brief      Read the touch status of all 13 channels as bit values in a 12
 * bit integer.
 *  @returns    a 12 bit integer with each bit corresponding to the touch status
 *              of a sensor. For example, if bit 0 is set then channel 0 of the
 * device is currently deemed to be touched.
 */
uint16_t mpr1212_get_touched(const struct device *dev)
{
    int ret;
    uint8_t t[2];

    ret = mpr121_seq_read(dev, MPR121_TOUCHSTATUS_L, t, 2);
    if (ret < 0)
    {
        LOG_ERR("mpr1212_get_touched: read status: %d", ret);
        return 0;
    }

    uint16_t ui16 = t[1] | (t[0] << 8);

    return ui16 & 0x0FFF;
}

static void mpr121_report_data(const struct device *dev)
{
    // const struct mpr121_config *config = dev->config;
    struct mpr121_data *data = dev->data;

    uint16_t touched = mpr1212_get_touched(dev);
    data->touch_status = touched;

    LOG_HEXDUMP_DBG(&touched, 1, "MPR121 Touch Status");

    // input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
    // input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);

    return;
}

static void mpr121_work_cb(struct k_work *work)
{
    LOG_DBG("Running mpr121_work_cb");
    struct mpr121_data *data = CONTAINER_OF(work, struct mpr121_data, work);
    mpr121_report_data(data->dev);
}

/*

static void mpr121_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct mpr121_data *data = CONTAINER_OF(cb, struct mpr121_data, gpio_cb);
    data->in_int = true;
    k_work_submit(&data->work);
}

static int mpr121_adc_sensitivity_reg_value(enum mpr121_sensitivity sensitivity)
{
    switch (sensitivity)
    {
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

static int mpr121_tune_edge_sensitivity(const struct device *dev)
{
    const struct mpr121_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = mpr121_era_read(dev, MPR121_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0)
    {
        LOG_WRN("Failed to read X val");
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = mpr121_era_read(dev, MPR121_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0)
    {
        LOG_WRN("Failed to read Y val");
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = mpr121_era_write(dev, MPR121_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0)
    {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = mpr121_era_write(dev, MPR121_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0)
    {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int mpr121_set_adc_tracking_sensitivity(const struct device *dev)
{
    const struct mpr121_config *config = dev->config;

    uint8_t val;
    int ret = mpr121_era_read(dev, MPR121_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0)
    {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= mpr121_adc_sensitivity_reg_value(config->sensitivity);

    ret = mpr121_era_write(dev, MPR121_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0)
    {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = mpr121_era_read(dev, MPR121_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0)
    {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int mpr121_force_recalibrate(const struct device *dev)
{
    uint8_t val;
    int ret = mpr121_seq_read(dev, MPR121_CAL_CFG, &val, 1);
    if (ret < 0)
    {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = mpr121_write(dev, MPR121_CAL_CFG, val);
    if (ret < 0)
    {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do
    {
        mpr121_seq_read(dev, MPR121_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int mpr121_set_sleep(const struct device *dev, bool enabled)
{
    uint8_t sys_cfg;
    int ret = mpr121_seq_read(dev, MPR121_SYS_CFG, &sys_cfg, 1);
    if (ret < 0)
    {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & MPR121_SYS_CFG_EN_SLEEP) != 0) == enabled)
    {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, MPR121_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = mpr121_write(dev, MPR121_SYS_CFG, sys_cfg);
    if (ret < 0)
    {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}

*/

int mpr121_get_config(const struct device *dev, uint8_t config1or2)
{
    int ret = 0;
    int regAddr = MPR121_CONFIG1;

    switch (config1or2)
    {
    case 1:
        regAddr = MPR121_CONFIG1;
        break;
    case 2:
        regAddr = MPR121_CONFIG2;
        break;
    default:
        LOG_ERR("mpr121_get_config: config1or2 can only have a value of 1 or 2");
        return -1;
    }

    uint8_t configValue[1];
    ret = mpr121_seq_read(dev, regAddr, configValue, 1);

    if (ret >= 0)
    {
        LOG_DBG("Default config%d: %d", config1or2, configValue[0]);
    }

    return configValue[0];
}

int mpr121_set_stop(const struct device *dev)
{
    int ret = 0;

    ret = mpr121_write(dev, MPR121_ECR, 0x00); // Stop mode
    if (ret < 0)
    {
        LOG_ERR("can't set stop mode %d", ret);
    }

    return ret;
}

int mpr121_set_run(const struct device *dev)
{
    // enable X electrodes and start MPR121
    uint8_t ECR_SETTING = 0b10000000 + 12; // 5 bits for baseline tracking & proximity disabled + X
                                           // amount of electrodes running (12)
    int ret = 0;

    ret = mpr121_write(dev, MPR121_ECR, ECR_SETTING); // Running mode
    if (ret < 0)
    {
        LOG_ERR("can't set run mode %d", ret);
    }

    return ret;
}

int mpr121_set_thresholds(const struct device *dev, uint8_t touch, uint8_t release)
{
    int ret = 0;

    // set all thresholds (the same)
    for (uint8_t i = 0; i < 12; i++)
    {

        ret = mpr121_write(dev, MPR121_TOUCHTH_0 + 2 * i, touch); // Set electrode touch threshold
        if (ret < 0)
            break;
        ret = mpr121_write(dev, MPR121_RELEASETH_0 + 2 * i, release); // Set electrode release threshold
        if (ret < 0)
            break;
    }

    if (ret < 0)
    {
        LOG_ERR("failed to set touch / release thresholds %d", ret);
    }

    return ret;
}

static int mpr121_init(const struct device *dev)
{
    struct mpr121_data *data = dev->data;
    // const struct mpr121_config *config = dev->config;
    int ret;

    LOG_DBG("Running MPR121 Init");

    // data->in_int = false;
    k_msleep(10);
    ret = mpr121_write(dev, MPR121_SOFTRESET, 0x63); // Soft Reset
    if (ret < 0)
    {
        LOG_ERR("can't soft reset %d", ret);
        return ret;
    }
    k_msleep(1);

    ret = mpr121_set_stop(dev);
    if (ret < 0)
    {
        return ret;
    }

    int config1 = mpr121_get_config(dev, 1);
    if (config1 != 0x24)
    {
        LOG_ERR("config1 is not 0x24, it is %d", config1);
        return -1;
    }

    ret = mpr121_set_thresholds(dev, MPR121_TOUCH_THRESHOLD, MPR121_RELEASE_THRESHOLD);
    if (ret < 0)
    {
        return ret;
    }

    mpr121_write(dev, MPR121_MHDR, 0x01);
    mpr121_write(dev, MPR121_NHDR, 0x01);
    mpr121_write(dev, MPR121_NCLR, 0x0E);
    mpr121_write(dev, MPR121_FDLR, 0x00);

    mpr121_write(dev, MPR121_MHDF, 0x01);
    mpr121_write(dev, MPR121_NHDF, 0x05);
    mpr121_write(dev, MPR121_NCLF, 0x01);
    mpr121_write(dev, MPR121_FDLF, 0x00);

    mpr121_write(dev, MPR121_NHDT, 0x00);
    mpr121_write(dev, MPR121_NCLT, 0x00);
    mpr121_write(dev, MPR121_FDLT, 0x00);

    mpr121_write(dev, MPR121_DEBOUNCE, 0);
    mpr121_write(dev, MPR121_CONFIG1, 0x10); // default, 16uA charge current
    mpr121_write(dev, MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

#if IS_ENABLED(CONFIG_MPR121_AUTOCONFIG)
    mpr121_write(dev, MPR121_AUTOCONFIG0, 0x0B);

    // correct values for Vdd = 3.3V
    mpr121_write(dev, MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
    mpr121_write(dev, MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
    mpr121_write(dev, MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65
#endif                                          // IS_ENABLED(CONFIG_MPR121_AUTOCONFIG)

    mpr121_set_run(dev); // Start MPR121

    // ret = mpr121_set_adc_tracking_sensitivity(dev);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to set ADC sensitivity %d", ret);
    //     return ret;
    // }

    // ret = mpr121_tune_edge_sensitivity(dev);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to tune edge sensitivity %d", ret);
    //     return ret;
    // }
    // ret = mpr121_force_recalibrate(dev);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to force recalibration %d", ret);
    //     return ret;
    // }

    // if (config->sleep_en)
    // {
    //     ret = mpr121_set_sleep(dev, true);
    //     if (ret < 0)
    //     {
    //         return ret;
    //     }
    // }

    // uint8_t packet[1];
    // ret = mpr121_seq_read(dev, MPR121_SLEEP_INTERVAL, packet, 1);

    // if (ret >= 0)
    // {
    //     LOG_DBG("Default sleep interval %d", packet[0]);
    // }

    // ret = mpr121_write(dev, MPR121_SLEEP_INTERVAL, 255);
    // if (ret <= 0)
    // {
    //     LOG_DBG("Failed to update sleep interaval %d", ret);
    // }

    // uint8_t feed_cfg2 = MPR121_FEED_CFG2_EN_IM | MPR121_FEED_CFG2_EN_BTN_SCRL;
    // if (config->no_taps)
    // {
    //     feed_cfg2 |= MPR121_FEED_CFG2_DIS_TAP;
    // }
    // if (config->rotate_90)
    // {
    //     feed_cfg2 |= MPR121_FEED_CFG2_ROTATE_90;
    // }
    // ret = mpr121_write(dev, MPR121_FEED_CFG2, feed_cfg2);
    // if (ret < 0)
    // {
    //     LOG_ERR("can't write %d", ret);
    //     return ret;
    // }
    // uint8_t feed_cfg1 = MPR121_FEED_CFG1_EN_FEED;
    // if (config->x_invert)
    // {
    //     feed_cfg1 |= MPR121_FEED_CFG1_INV_X;
    // }

    // if (config->y_invert)
    // {
    //     feed_cfg1 |= MPR121_FEED_CFG1_INV_Y;
    // }
    // if (feed_cfg1)
    // {
    //     ret = mpr121_write(dev, MPR121_FEED_CFG1, feed_cfg1);
    // }
    // if (ret < 0)
    // {
    //     LOG_ERR("can't write %d", ret);
    //     return ret;
    // }

    data->dev = dev;

    // mpr121_clear_status(dev);

    // gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    // gpio_init_callback(&data->gpio_cb, mpr121_gpio_cb, BIT(config->dr.pin));
    // ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    // if (ret < 0)
    // {
    //     LOG_ERR("Failed to set DR callback: %d", ret);
    //     return -EIO;
    // }

    k_work_init(&data->work, mpr121_work_cb);

    // mpr121_write(dev, MPR121_FEED_CFG1, feed_cfg1);

    // set_int(dev, true);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int mpr121_pm_action(const struct device *dev, enum pm_device_action action)
{
    // switch (action)
    // {
    // case PM_DEVICE_ACTION_SUSPEND:
    //     return set_int(dev, false);
    // case PM_DEVICE_ACTION_RESUME:
    //     return set_int(dev, true);
    // default:
    //     return -ENOTSUP;
    // }

    return -ENOTSUP;
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

// #define MPR121_INST(n)                                                                \
//     static struct mpr121_data mpr121_data_##n;                                        \
//     static const struct mpr121_config mpr121_config_##n = {                           \
//         .bus = {.i2c = I2C_DT_SPEC_INST_GET(n)},                                      \
//         .seq_read = mpr121_i2c_seq_read,                                              \
//         .write = mpr121_i2c_write,                                                    \
//     };                                                                                \
//     PM_DEVICE_DT_INST_DEFINE(n, mpr121_pm_action);                                    \
//     DEVICE_DT_INST_DEFINE(n, mpr121_init, PM_DEVICE_DT_INST_GET(n), &mpr121_data_##n, \
//                           &mpr121_config_##n, POST_KERNEL, CONFIG_INPUT_MPR121_INIT_PRIORITY, NULL);

// DT_INST_FOREACH_STATUS_OKAY(MPR121_INST)

#define MPR121_INST(n)                                                                \
    static struct mpr121_data mpr121_data_##n;                                        \
    static const struct mpr121_config mpr121_config_##n = {                           \
        .bus = I2C_DT_SPEC_INST_GET(n),                                               \
        .seq_read = mpr121_i2c_seq_read,                                              \
        .write = mpr121_i2c_write,                                                    \
    };                                                                                \
    PM_DEVICE_DT_INST_DEFINE(n, mpr121_pm_action);                                    \
    DEVICE_DT_INST_DEFINE(n, mpr121_init, PM_DEVICE_DT_INST_GET(n), &mpr121_data_##n, \
                          &mpr121_config_##n, POST_KERNEL, CONFIG_INPUT_MPR121_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MPR121_INST)