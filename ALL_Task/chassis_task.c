#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "../Components/remote/remote.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"

/* --- 逻辑常量与控制参数 --- */
#define GIMBAL_YAW_SENS         0.010f
#define GIMBAL_PIT_SENS         0.002f
#define MOUSE_YAW_SENS          0.0004f  // 鼠标横向灵敏度
#define MOUSE_PIT_SENS          0.0002f  // 鼠标纵向灵敏度
#define FOLLOW_P_GAIN           0.5f
#define RC_DEADZONE             10
#define YAW_CENTER_OFFSET       1.9f//-1.7f（步兵） //1.9f（哨兵）

// 底盘几何参数配置
#define MOTOR_RPM_TO_VECTOR     3000.0f
#define CHASSIS_MAX_RAD         60.0f

// 三档速度配置（可按实车手感直接调参）
#define CHASSIS_SPEED_GEAR_LOW   0.8f
#define CHASSIS_SPEED_GEAR_MID   1.1f
#define CHASSIS_SPEED_GEAR_HIGH  2.0f

// 自瞄周期性机动参数
#define AUTO_AIM_FORCE_SPIN_INTERVAL_MS   4000U // 自瞄中每隔 4s 触发一次机动
#define AUTO_AIM_FORCE_SPIN_DURATION_MS    300U // 机动持续 0.2s
#define AUTO_AIM_FORCE_SWAY_HALF_MS        150U // 前 0.1s 左移，后 0.1s 右移
#define AUTO_AIM_FORCE_SWAY_SPEED         CHASSIS_SPEED_GEAR_LOW

// 自瞄模式自动提档条件（capacity_voltage 单位：*100）
#define AUTO_AIM_HIGH_GEAR_HURT_CAP_V      1800  // 受击触发阈值：> 18.0V
#define AUTO_AIM_HIGH_GEAR_FULL_CAP_V      2550  // 常规触发阈值：> 25.5V
#define AUTO_AIM_HIGH_GEAR_STOP_CAP_V      1000  // 低于 10.0V 立即退出高速档
#define AUTO_AIM_HIGH_GEAR_MAX_MS         5000U // 单次高速档最长持续 10s

// 超级电容低压锁档阈值（capacity_voltage 单位：*100）
#define CAP_VOLT_LOW_GEAR_THRESHOLD       1000  // <= 10.0V 强制最低档，> 10.0V 立即解除

// 回正相关参数
#define YAW_ALIGN_THRESHOLD     0.05f    // 放宽到位阈值（适配机械误差，约2.86度）
#define WHEEL_ACTIVE_THRESHOLD  0.01f    // 拨轮有效输入阈值
#define QE_ACTIVE_THRESHOLD     0.01f     // Q/E有效输入阈值

// 底盘渐加速参数（单位：归一化速度/秒）
#define CHASSIS_VX_ACCEL_UP      2.2f
#define CHASSIS_VX_ACCEL_DOWN    4.2f
#define CHASSIS_VY_ACCEL_UP      2.2f
#define CHASSIS_VY_ACCEL_DOWN    4.2f
#define CHASSIS_VW_ACCEL_UP      2.5f
#define CHASSIS_VW_ACCEL_DOWN    5.0f

// 裁判比赛阶段：4 为比赛进行中（开赛）
#define GAME_PROGRESS_BATTLE      4U
#define LED_OFFLINE_BLINK_MS      200U

/* --- 静态控制变量 --- */
static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static float vx_ramp = 0.0f, vy_ramp = 0.0f;
static float vw_ramp = 0.0f;

static uint32_t last_rc_tick = 0;

// 扩展：新增Q/E相关状态变量，和拨轮统一管理
static uint8_t last_wheel_active = 0;    // 上一帧拨轮是否激活
static uint8_t last_qe_active = 0;       // 上一帧Q/E是否激活
static uint8_t yaw_align_enable = 0;     // 回正使能标志（1=需要回正，0=不需要）
static float last_manual_vw = 0.0f;      // 保存「拨轮/Q/E」松开前的最后有效旋转速度（统一变量，避免冲突）
// 新增：Q/E 按键切换状态与防抖记录（用于实现按键切换而非持续按住）
static uint8_t left_rotate_toggle = 0;   // Q键切换：左旋状态（1=左旋开启）
static uint8_t right_rotate_toggle = 0;  // E键切换：右旋状态（1=右旋开启）
static uint8_t last_q_pressed = 0;       // 上一帧 Q 键状态（防抖）
static uint8_t last_e_pressed = 0;       // 上一帧 E 键状态（防抖）
static uint8_t cap_low_gear_lock = 0;    // 超级电容低压锁档（滞回）
static uint8_t auto_aim_high_gear_latch = 0; // 自瞄模式自动高速档锁存
static uint32_t auto_aim_high_gear_until_tick = 0U; // 自瞄高速档截止时刻
static uint8_t last_auto_aim_high_voltage_ok = 0U; // 上一帧是否满足自瞄高压触发条件
static uint8_t last_auto_aim_high_gear_active = 0U; // 上一帧自瞄是否处于高速档
static uint8_t auto_aim_force_spin_active = 0U; // 自瞄周期机动状态
static uint32_t auto_aim_force_spin_start_tick = 0U; // 自瞄周期机动开始时刻
static uint32_t auto_aim_force_spin_next_tick = 0U; // 下一次自瞄周期机动触发时刻
static uint16_t last_hp = 0U;            // 上一帧血量，用于检测受击
static uint8_t hp_initialized = 0U;      // 血量初始化标志

static struct uart_device *log_uart = NULL;

#define LOG_PRINT(...) do { if (log_uart != NULL) { log_uart->Print(log_uart, __VA_ARGS__); } } while (0)
#define CHASSIS_VERBOSE_LOG          0U
#define CHASSIS_STATE_LOG_PERIOD_MS  30U
#define LOG_VERBOSE_PRINT(...) do { if (CHASSIS_VERBOSE_LOG) { LOG_PRINT(__VA_ARGS__); } } while (0)

static float Rad_Format(float angle) {
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

static float Chassis_Slew_Limit(float target, float current, float accel_up, float accel_down, float dt_s)
{
    float delta = target - current;
    float max_step;

    // 反向或减速时用更大的下坡斜率，保证松手后不拖沓
    if ((target * current < 0.0f) || (fabsf(target) < fabsf(current))) {
        max_step = accel_down * dt_s;
    } else {
        max_step = accel_up * dt_s;
    }

    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return current + delta;
}

static void Chassis_Update_Status_LED(void)
{
    static uint32_t last_blink_tick = 0U;
    static uint8_t red_on = 0U;
    uint32_t now = osKernelSysTick();

    // 掉线最高优先级：红灯闪烁
    if (!robot_ctrl.monitor.remote_online) {
        if ((uint32_t)(now - last_blink_tick) >= LED_OFFLINE_BLINK_MS) {
            red_on ^= 1U;
            last_blink_tick = now;
        }

        LED_BLUE_RESET();
        LED_GREEN_RESET();
        if (red_on) {
            LED_RED_SET();
        } else {
            LED_RED_RESET();
        }
        return;
    }

    // 在线状态：开赛绿灯，未开赛红灯
    uint8_t game_started = (robot_ctrl.game_info.online_301 &&
                            (robot_ctrl.game_info.game_progress == GAME_PROGRESS_BATTLE)) ? 1U : 0U;

    LED_BLUE_RESET();
    if (game_started) {
        LED_GREEN_SET();
        LED_RED_RESET();
    } else {
        LED_GREEN_RESET();
        LED_RED_SET();
    }
}

void chassis_task_func(void const * argument) {
    /******************************************************************************************************************/
    /* 初始化 */

    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    const RC_ctrl_t *rc = robot_ctrl.rc;
    float wheel_targets[4] = {0};
    uint8_t last_remote_online = 0xFFU;
    uint32_t last_diag_tick = 0U;
    uint32_t last_ctrl_tick = 0U;

    /******************************************************************************************************************/
    // 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();
        Chassis_Update_Status_LED();
        float dt_s = 0.002f;
        if (last_ctrl_tick != 0U) {
            uint32_t dt_ms = (uint32_t)(current_tick - last_ctrl_tick);
            if (dt_ms == 0U) dt_ms = 1U;
            if (dt_ms > 20U) dt_ms = 20U;
            dt_s = (float)dt_ms * 0.001f;
        }
        last_ctrl_tick = current_tick;

        if (log_uart == NULL) {
            log_uart = uart_get_device("uart1_dma");
        }

        /**************************************************************************************************************/
        // 遥控器掉线检测：使用快照+有符号差值，避免与中断并发更新导致的无符号下溢误判
        uint32_t rc_last_tick = rc->vt13.last_update_tick;
        int32_t rc_tick_diff = (int32_t)(current_tick - rc_last_tick);
        if (rc_tick_diff > 1000) {
            robot_ctrl.monitor.remote_online = 1U;
            robot_ctrl.monitor.system_enabled = 1U;
            robot_ctrl.monitor.plan_enabled = 1U;
            robot_ctrl.chassis_mode = CHASSIS_FOLLOW;

            // 掉线时重置所有标志和保存的速度
            yaw_align_enable = 0;
            last_wheel_active = 0;
            last_qe_active = 0;
            last_manual_vw = 0.0f;
            // 清除 Q/E 切换态与按键防抖，避免断线后滞留旋转状态
            left_rotate_toggle = 0;
            right_rotate_toggle = 0;
            last_q_pressed = 0;
            last_e_pressed = 0;
            last_auto_aim_high_gear_active = 0U;
            auto_aim_force_spin_active = 0U;
            auto_aim_force_spin_start_tick = 0U;
            auto_aim_force_spin_next_tick = 0U;
            vx_ramp = 0.0f;
            vy_ramp = 0.0f;
            vw_ramp = 0.0f;

            if (last_remote_online != 0U) {
                LOG_VERBOSE_PRINT("[CHS][TIMEOUT] t=%lu last_rc=%lu dt=%ld\r\n",
                                  (unsigned long)current_tick,
                                  (unsigned long)rc_last_tick,
                                  (long)rc_tick_diff);
            }
        } else {
            robot_ctrl.monitor.remote_online = 1;
            robot_ctrl.monitor.system_enabled = 1U;
            robot_ctrl.monitor.plan_enabled = 1U;
            robot_ctrl.chassis_mode = CHASSIS_FOLLOW;
        }

        if (last_remote_online != robot_ctrl.monitor.remote_online) {
            LOG_VERBOSE_PRINT("[CHS][REMOTE] t=%lu online=%u\r\n",
                              (unsigned long)current_tick,
                              (unsigned int)robot_ctrl.monitor.remote_online);
            last_remote_online = robot_ctrl.monitor.remote_online;
        }

        if ((uint32_t)(current_tick - last_diag_tick) >= CHASSIS_STATE_LOG_PERIOD_MS) {
            uint32_t ok_cnt = 0U, bad_len_cnt = 0U;
            RC_Get_VT13_RxDiag(&ok_cnt, &bad_len_cnt);
            // 心跳也基于快照差值，避免并发读写造成显示为 0xFFFFFFFF
            uint32_t hb_last_tick = rc->vt13.last_update_tick;
            int32_t hb_tick_diff = (int32_t)(current_tick - hb_last_tick);
            LOG_VERBOSE_PRINT("[CHS][HB] t=%lu remote=%u sys=%u plan=%u rc_dt=%lu rx_ok=%lu rx_bad=%lu\r\n",
                              (unsigned long)current_tick,
                              (unsigned int)robot_ctrl.monitor.remote_online,
                              (unsigned int)robot_ctrl.monitor.system_enabled,
                              (unsigned int)robot_ctrl.monitor.plan_enabled,
                              (unsigned long)((hb_tick_diff >= 0) ? hb_tick_diff : 0),
                              (unsigned long)ok_cnt,
                              (unsigned long)bad_len_cnt);
            last_diag_tick = current_tick;
        }

        /**************************************************************************************************************/

        if (robot_ctrl.monitor.remote_online) {
            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) {
                    // --- A. 输入源融合 (遥控器摇杆 + 键盘) ---
                    float vx_rc = (abs(rc->vt13.rc_vt13.ch[0]) > RC_DEADZONE) ? ((float)rc->vt13.rc_vt13.ch[0] / 660.0f) : 0.0f;
                    float vy_rc = (abs(rc->vt13.rc_vt13.ch[1]) > RC_DEADZONE) ? ((float)rc->vt13.rc_vt13.ch[1] / 660.0f) : 0.0f;
                    float vw_rc = (abs(rc->vt13.rc_vt13.wheel) > RC_DEADZONE) ? ((float)rc->vt13.rc_vt13.wheel / 660.0f) : 0.0f;

                    float vx_kb = 0.0f, vy_kb = 0.0f, vw_kb = 0.0f;
                    float speed_ratio;
                    float motion_limit_ratio;
                    float auto_spin_speed_ratio;
                    uint8_t auto_spin_active = (robot_ctrl.gimbal_mode == GIMBAL_AUTO) ? 1U : 0U;
                    uint8_t got_hurt_now = 0U;

                    // 三档仲裁：低压锁最低档 > Shift最高档 > 默认中档
                    if (robot_ctrl.game_info.online_301) {
                        int16_t cap_v = robot_ctrl.game_info.capacity_voltage;

                        if (!hp_initialized) {
                            last_hp = robot_ctrl.game_info.current_HP;
                            hp_initialized = 1U;
                        } else {
                            if (robot_ctrl.game_info.current_HP < last_hp) {
                                got_hurt_now = 1U;
                            }
                            last_hp = robot_ctrl.game_info.current_HP;
                        }

                        if (cap_v <= CAP_VOLT_LOW_GEAR_THRESHOLD) {
                            cap_low_gear_lock = 1U;
                        } else {
                            cap_low_gear_lock = 0U;
                        }
                    } else {
                        // 无有效电容电压时不强制限速，避免默认0值导致长期锁慢档
                        cap_low_gear_lock = 0U;
                        auto_aim_high_gear_latch = 0U;
                        auto_aim_high_gear_until_tick = 0U;
                        last_auto_aim_high_voltage_ok = 0U;
                        hp_initialized = 0U;
                    }

                    if (cap_low_gear_lock) {
                        speed_ratio = CHASSIS_SPEED_GEAR_LOW;
                        auto_aim_high_gear_latch = 0U;
                        auto_aim_high_gear_until_tick = 0U;
                        last_auto_aim_high_voltage_ok = 0U;
                    } else if (auto_spin_active && robot_ctrl.game_info.online_301) {
                        int16_t cap_v = robot_ctrl.game_info.capacity_voltage;
                        uint8_t high_voltage_now = (cap_v > AUTO_AIM_HIGH_GEAR_FULL_CAP_V) ? 1U : 0U;
                        uint8_t high_voltage_trigger = (high_voltage_now && !last_auto_aim_high_voltage_ok) ? 1U : 0U;

                        if (auto_aim_high_gear_latch) {
                            if ((cap_v < AUTO_AIM_HIGH_GEAR_STOP_CAP_V) ||
                                ((int32_t)(current_tick - auto_aim_high_gear_until_tick) >= 0)) {
                                auto_aim_high_gear_latch = 0U;
                                auto_aim_high_gear_until_tick = 0U;
                            }
                        }

                        if (!auto_aim_high_gear_latch &&
                            (high_voltage_trigger ||
                             (got_hurt_now && (cap_v > AUTO_AIM_HIGH_GEAR_HURT_CAP_V)))) {
                            auto_aim_high_gear_latch = 1U;
                            auto_aim_high_gear_until_tick = current_tick + AUTO_AIM_HIGH_GEAR_MAX_MS;
                        }

                        last_auto_aim_high_voltage_ok = high_voltage_now;
                        speed_ratio = auto_aim_high_gear_latch ? CHASSIS_SPEED_GEAR_HIGH : CHASSIS_SPEED_GEAR_MID;
                    } else if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_SHIFT)) {
                        speed_ratio = CHASSIS_SPEED_GEAR_HIGH;
                        auto_aim_high_gear_latch = 0U;
                        auto_aim_high_gear_until_tick = 0U;
                        last_auto_aim_high_voltage_ok = 0U;
                    } else {
                        speed_ratio = CHASSIS_SPEED_GEAR_MID;
                        auto_aim_high_gear_latch = 0U;
                        auto_aim_high_gear_until_tick = 0U;
                        last_auto_aim_high_voltage_ok = 0U;
                    }

                    if (!auto_spin_active || auto_aim_high_gear_latch) {
                        auto_aim_force_spin_active = 0U;
                        auto_aim_force_spin_start_tick = 0U;
                        auto_aim_force_spin_next_tick = 0U;
                    } else {
                        uint8_t high_to_low_trigger = last_auto_aim_high_gear_active ? 1U : 0U;

                        if (auto_aim_force_spin_active) {
                            if ((uint32_t)(current_tick - auto_aim_force_spin_start_tick) >= AUTO_AIM_FORCE_SPIN_DURATION_MS) {
                                auto_aim_force_spin_active = 0U;
                                auto_aim_force_spin_start_tick = 0U;
                                auto_aim_force_spin_next_tick = current_tick + AUTO_AIM_FORCE_SPIN_INTERVAL_MS;
                            }
                        }

                        if (!auto_aim_force_spin_active) {
                            if (high_to_low_trigger) {
                                auto_aim_force_spin_active = 1U;
                                auto_aim_force_spin_start_tick = current_tick;
                                auto_aim_force_spin_next_tick = current_tick + AUTO_AIM_FORCE_SPIN_INTERVAL_MS;
                            } else if (auto_aim_force_spin_next_tick == 0U) {
                                auto_aim_force_spin_next_tick = current_tick + AUTO_AIM_FORCE_SPIN_INTERVAL_MS;
                            } else if ((int32_t)(current_tick - auto_aim_force_spin_next_tick) >= 0) {
                                auto_aim_force_spin_active = 1U;
                                auto_aim_force_spin_start_tick = current_tick;
                                auto_aim_force_spin_next_tick = current_tick + AUTO_AIM_FORCE_SPIN_INTERVAL_MS;
                            }
                        }
                    }

                    last_auto_aim_high_gear_active = (auto_spin_active && auto_aim_high_gear_latch) ? 1U : 0U;

                    auto_spin_speed_ratio = speed_ratio;
                    motion_limit_ratio = speed_ratio;

                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_W)) vy_kb += speed_ratio;
                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_S)) vy_kb -= speed_ratio;
                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_A)) vx_kb -= speed_ratio;
                    if (KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_D)) vx_kb += speed_ratio;

                    // Q/E: 切换式按键（按一次切换左旋/右旋状态），使用上升沿检测实现防抖
                    uint8_t q_pressed = KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_Q);
                    uint8_t e_pressed = KEY_PRESSED(rc->vt13.key_vt13.v, KEY_VT13_E);
                    uint8_t q_trigger = (q_pressed && !last_q_pressed); // Q 上升沿
                    uint8_t e_trigger = (e_pressed && !last_e_pressed); // E 上升沿

                    if (q_trigger) {
                        left_rotate_toggle = !left_rotate_toggle;    // 切换左旋状态
                        if (left_rotate_toggle) right_rotate_toggle = 0; // 互斥，打开左则关闭右
                    }
                    if (e_trigger) {
                        right_rotate_toggle = !right_rotate_toggle;  // 切换右旋状态
                        if (right_rotate_toggle) left_rotate_toggle = 0; // 互斥，打开右则关闭左
                    }

                    // 根据切换状态设置 vw_kb 为固定手动速度（与 speed_ratio 同量级），或保持为 0
                    if (left_rotate_toggle) vw_kb = -speed_ratio;
                    else if (right_rotate_toggle) vw_kb = speed_ratio;


                    uint8_t upper_ctrl_enabled = (robot_ctrl.monitor.plan_enabled &&
                                                  robot_ctrl.game_info.online_301 &&
                                                  (robot_ctrl.game_info.game_progress == GAME_PROGRESS_BATTLE)) ? 1U : 0U;

                    // 比赛开始后才允许受上位机目标状态影响
                    if (upper_ctrl_enabled && (robot_ctrl.target_info.valid == 1U)) {
                        vw_kb = speed_ratio;
                        yaw_align_enable = 0U;
                    }

                    // 自瞄模式联动底盘自转：进入 GIMBAL_AUTO 后底盘持续右旋。

                    // 更新上一帧按键状态（防抖记录）
                    last_q_pressed = q_pressed;
                    last_e_pressed = e_pressed;

                    // 路径规划速度仅在比赛开始后生效，其他时间忽略上位机
                    float vx_plan = upper_ctrl_enabled ? robot_ctrl.chassis.cmd_vx : 0.0f;
                    float vy_plan = upper_ctrl_enabled ? robot_ctrl.chassis.cmd_vy : 0.0f;

                    float total_vx = vx_rc + vx_kb - vy_plan;
                    float total_vy = vy_rc + vy_kb + vx_plan;

                    if (auto_spin_active && auto_aim_force_spin_active) {
                        uint32_t force_elapsed = (uint32_t)(current_tick - auto_aim_force_spin_start_tick);
                        total_vx = (force_elapsed < AUTO_AIM_FORCE_SWAY_HALF_MS) ? -AUTO_AIM_FORCE_SWAY_SPEED : AUTO_AIM_FORCE_SWAY_SPEED;
                        total_vy = 0.0f;
                    }

                    // --- B. 各向同性限速 ---
                    float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                    if (v_norm > motion_limit_ratio) {
                        total_vx = total_vx / v_norm * motion_limit_ratio;
                        total_vy = total_vy / v_norm * motion_limit_ratio;
                    }

                    // --- B2. 渐加速/渐减速 ---
                    if (auto_spin_active && auto_aim_force_spin_active) {
                        vx_ramp = total_vx;
                        vy_ramp = total_vy;
                    } else {
                        vx_ramp = Chassis_Slew_Limit(total_vx, vx_ramp, CHASSIS_VX_ACCEL_UP, CHASSIS_VX_ACCEL_DOWN, dt_s);
                        vy_ramp = Chassis_Slew_Limit(total_vy, vy_ramp, CHASSIS_VY_ACCEL_UP, CHASSIS_VY_ACCEL_DOWN, dt_s);
                    }

                    // --- C. 跟随与旋转逻辑（扩展：Q/E+拨轮统一回正）---
                    float yaw_m_pos;
                    yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                    float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                    // 步骤1：判断当前拨轮、Q/E是否处于激活状态
                    uint8_t current_wheel_active = (fabsf(vw_rc) > WHEEL_ACTIVE_THRESHOLD) ? 1 : 0;
                    uint8_t current_qe_active = (fabsf(vw_kb) > QE_ACTIVE_THRESHOLD) ? 1 : 0;
                    // 合并手动输入状态（拨轮或Q/E有一个激活，就认为是手动控制阶段）
                    uint8_t current_manual_active = current_wheel_active || current_qe_active;

                    // 步骤2：手动控制阶段，实时保存最后有效旋转速度（统一保存到last_manual_vw）
                    if (current_manual_active) {
                        last_manual_vw = vw_rc + vw_kb; // 保存当前拨轮+Q/E的合成速度（符合原有手动逻辑）
                    }

                    // 步骤3：检测「拨轮」或「Q/E」的松开下降沿，触发回正（二选一触发，避免冲突）
                    uint8_t wheel_release_trigger = (last_wheel_active && !current_wheel_active && !current_qe_active);
                    uint8_t qe_release_trigger = (last_qe_active && !current_qe_active && !current_wheel_active);
                    if ((wheel_release_trigger || qe_release_trigger) && !yaw_align_enable) {
                        yaw_align_enable = 1; // 开启回正使能
                    }

                    // 步骤4：优先级排序：手动控制 > 固定速度回正 > 正常跟随
                    float vw_final = 0;
                    if (current_manual_active) {
                        // 手动控制阶段：关闭回正使能，优先响应输入
                        yaw_align_enable = 0;
                        vw_final = vw_rc + vw_kb;
                    } else if (yaw_align_enable) {
                        // 自动回正阶段：直接使用松开前保存的最后手动速度，固定速度回正
                        vw_final = last_manual_vw;

                        // 回正到位判断，到位后清零所有标志和速度
                        if (fabsf(angle_error) < YAW_ALIGN_THRESHOLD) {
                            yaw_align_enable = 0;
                            vw_final = 0;
                            last_manual_vw = 0.0f;
                        }
                    } else {
                        // 正常跟随阶段：原有的云台跟随逻辑
                        vw_final = -angle_error * FOLLOW_P_GAIN;
                    }

                    if (auto_spin_active) {
                        yaw_align_enable = 0U;
                        vw_final = auto_spin_speed_ratio;
                    }

                    vw_ramp = Chassis_Slew_Limit(vw_final, vw_ramp, CHASSIS_VW_ACCEL_UP, CHASSIS_VW_ACCEL_DOWN, dt_s);
                    vw_final = vw_ramp;

                    // 步骤5：更新上一帧状态记录（供下一帧边缘检测使用）
                    last_wheel_active = current_wheel_active;
                    last_qe_active = current_qe_active;

                    robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                    // --- D. 随动坐标系变换 ---
                    float final_vx = vx_ramp * cosf(angle_error) - vy_ramp * sinf(angle_error);
                    float final_vy = vx_ramp * sinf(angle_error) + vy_ramp * cosf(angle_error);

                    // --- E. 逆运动学计算 ---
                    wheel_targets[0] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[1] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[2] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[3] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

                    for (int i = 0; i < 4; i++) {
                        if (chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                    }
                }
            } else {
                vx_ramp = 0.0f;
                vy_ramp = 0.0f;
                vw_ramp = 0.0f;
            }
        }
        else {
            // 遥控器掉线：仅清零运动，灯效由 Chassis_Update_Status_LED 统一管理
            vx_ramp = 0.0f;
            vy_ramp = 0.0f;
            vw_ramp = 0.0f;
            osDelay(100);
        }

        osDelay(2);
    }
}

