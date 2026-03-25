#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include "struct_typedef.h"

#define CHASSIS_OMNI_WHEEL_COUNT 4U

typedef struct {
    int8_t vx_sign;
    int8_t vy_sign;
    int8_t vw_sign;
    int8_t motor_sign;
} chassis_omni_wheel_map_t;

typedef struct {
    float rpm_scale;
    chassis_omni_wheel_map_t wheels[CHASSIS_OMNI_WHEEL_COUNT];
} chassis_omni_config_t;

void chassis_omni_calc_wheel_targets(const chassis_omni_config_t *config,
                                     float vx,
                                     float vy,
                                     float vw,
                                     float wheel_targets[CHASSIS_OMNI_WHEEL_COUNT]);

#endif
