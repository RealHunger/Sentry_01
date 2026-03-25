#ifndef RC_INPUT_COMPAT_H
#define RC_INPUT_COMPAT_H

#include "../Components/remote/remote.h"

#define RC_INPUT_ENABLE_DT7 0U

typedef enum {
    RC_INPUT_NONE = 0,
    RC_INPUT_VT13,
    RC_INPUT_DT7,
} rc_input_source_e;

static inline uint8_t rc_input_source_online(uint32_t now, uint32_t last_tick, uint32_t timeout_ms)
{
    return ((uint32_t)(now - last_tick) <= timeout_ms) ? 1U : 0U;
}

static inline rc_input_source_e rc_input_get_source(const RC_ctrl_t *rc, uint32_t now, uint32_t timeout_ms)
{
    if (rc == 0) {
        return RC_INPUT_NONE;
    }

    if (rc_input_source_online(now, rc->vt13.last_update_tick, timeout_ms)) {
        return RC_INPUT_VT13;
    }

    #if RC_INPUT_ENABLE_DT7
    if (rc_input_source_online(now, rc->dt7.last_update_tick, timeout_ms)) {
        return RC_INPUT_DT7;
    }
    #endif

    return RC_INPUT_NONE;
}

static inline uint32_t rc_input_get_last_update_tick(const RC_ctrl_t *rc, uint32_t now, uint32_t timeout_ms)
{
    rc_input_source_e source = rc_input_get_source(rc, now, timeout_ms);

    if (source == RC_INPUT_VT13) {
        return rc->vt13.last_update_tick;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.last_update_tick;
    }
    #endif

    return 0U;
}

static inline int16_t rc_input_get_ch(const RC_ctrl_t *rc, rc_input_source_e source, uint8_t index)
{
    if ((rc == 0) || (index >= 4U)) {
        return 0;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.rc_vt13.ch[index];
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.rc_dt7.ch[index];
    }
    #endif

    return 0;
}

static inline int16_t rc_input_get_wheel(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (rc == 0) {
        return 0;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.rc_vt13.wheel;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.rc_dt7.wheel;
    }
    #endif

    return 0;
}

static inline uint16_t rc_input_get_keys(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (rc == 0) {
        return 0U;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.key_vt13.v;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.key_dt7.v;
    }
    #endif

    return 0U;
}

static inline int16_t rc_input_get_mouse_x(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (rc == 0) {
        return 0;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.mouse_vt13.x;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.mouse_dt7.x;
    }
    #endif

    return 0;
}

static inline int16_t rc_input_get_mouse_y(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (rc == 0) {
        return 0;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.mouse_vt13.y;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.mouse_dt7.y;
    }
    #endif

    return 0;
}

static inline uint8_t rc_input_mouse_l(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (rc == 0) {
        return 0U;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.mouse_vt13.press_l ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.mouse_dt7.press_l ? 1U : 0U;
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_mouse_r(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (rc == 0) {
        return 0U;
    }

    if (source == RC_INPUT_VT13) {
        return rc->vt13.mouse_vt13.press_r ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc->dt7.mouse_dt7.press_r ? 1U : 0U;
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_mouse_m(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if ((rc != 0) && (source == RC_INPUT_VT13)) {
        return rc->vt13.mouse_vt13.press_m ? 1U : 0U;
    }

    return 0U;
}

static inline uint8_t rc_input_pause_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if ((rc != 0) && (source == RC_INPUT_VT13)) {
        return rc->vt13.rc_vt13.pause ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        uint16_t keys = rc_input_get_keys(rc, source);
        return KEY_PRESSED(keys, KEY_DT7_CTRL);
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_enable_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (source == RC_INPUT_VT13) {
        return KEY_PRESSED(rc_input_get_keys(rc, source), KEY_VT13_C);
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return KEY_PRESSED(rc_input_get_keys(rc, source), KEY_DT7_C);
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_disable_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (source == RC_INPUT_VT13) {
        return KEY_PRESSED(rc_input_get_keys(rc, source), KEY_VT13_X);
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return KEY_PRESSED(rc_input_get_keys(rc, source), KEY_DT7_X);
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_mode_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (source == RC_INPUT_VT13) {
        return (uint8_t)(((rc != 0) && rc->vt13.rc_vt13.custom_l) || rc_input_mouse_r(rc, source));
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return (uint8_t)(KEY_PRESSED(rc_input_get_keys(rc, source), KEY_DT7_G) || rc_input_mouse_r(rc, source));
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_plan_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if ((rc != 0) && (source == RC_INPUT_VT13)) {
        return rc->vt13.rc_vt13.custom_r ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return KEY_PRESSED(rc_input_get_keys(rc, source), KEY_DT7_R);
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_trigger_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if ((rc != 0) && (source == RC_INPUT_VT13)) {
        return rc->vt13.rc_vt13.trigger ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if (source == RC_INPUT_DT7) {
        return rc_input_mouse_l(rc, source);
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_shoot_switch_ready(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if ((rc != 0) && (source == RC_INPUT_VT13)) {
        return (rc->vt13.rc_vt13.sw == RC_SW_S_VT13) ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if ((rc != 0) && (source == RC_INPUT_DT7)) {
        return switch_is_up(rc->dt7.rc_dt7.sw_l) ? 1U : 0U;
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_shoot_switch_stop(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if ((rc != 0) && (source == RC_INPUT_VT13)) {
        return (rc->vt13.rc_vt13.sw == RC_SW_C_VT13) ? 1U : 0U;
    }

    #if RC_INPUT_ENABLE_DT7
    if ((rc != 0) && (source == RC_INPUT_DT7)) {
        return switch_is_down(rc->dt7.rc_dt7.sw_l) ? 1U : 0U;
    }
    #endif

    return 0U;
}

static inline uint8_t rc_input_reverse_btn(const RC_ctrl_t *rc, rc_input_source_e source)
{
    if (source == RC_INPUT_VT13) {
        return rc_input_mouse_m(rc, source);
    }

    #if RC_INPUT_ENABLE_DT7
    if ((rc != 0) && (source == RC_INPUT_DT7)) {
        return switch_is_down(rc->dt7.rc_dt7.sw_r) ? 1U : 0U;
    }
    #endif

    return 0U;
}

#endif
