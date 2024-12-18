

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include "input_mpr121.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mpr121_sleeper, CONFIG_INPUT_LOG_LEVEL);

#define GET_MPR121(node_id) DEVICE_DT_GET(node_id),

static const struct device *mpr121_devs[] = {
    DT_FOREACH_STATUS_OKAY(nxp_mpr121, GET_MPR121)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool sleep = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 0 : 1;
    for (size_t i = 0; i < ARRAY_SIZE(mpr121_devs); i++) {
        mpr121_set_sleep(mpr121_devs[i], sleep);
    }

    return 0;
}

ZMK_LISTENER(zmk_mpr121_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_mpr121_idle_sleeper, zmk_activity_state_changed);