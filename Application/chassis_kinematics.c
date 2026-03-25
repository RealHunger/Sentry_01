#include "chassis_kinematics.h"

static int8_t chassis_normalize_sign(int8_t sign)
{
    return (sign >= 0) ? 1 : -1;
}

void chassis_omni_calc_wheel_targets(const chassis_omni_config_t *config,
                                     float vx,
                                     float vy,
                                     float vw,
                                     float wheel_targets[CHASSIS_OMNI_WHEEL_COUNT])
{
    if ((config == 0) || (wheel_targets == 0)) {
        return;
    }

    for (uint32_t i = 0; i < CHASSIS_OMNI_WHEEL_COUNT; ++i) {
        const chassis_omni_wheel_map_t *wheel = &config->wheels[i];
        float standard_target = (float)chassis_normalize_sign(wheel->vx_sign) * vx +
                                (float)chassis_normalize_sign(wheel->vy_sign) * vy +
                                (float)chassis_normalize_sign(wheel->vw_sign) * vw;

        wheel_targets[i] = standard_target * config->rpm_scale *
                           (float)chassis_normalize_sign(wheel->motor_sign);
    }
}
