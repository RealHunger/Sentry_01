#include "motor_task.h"
#include "cmsis_os.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"
#include "stdio.h"

#define MOTOR_OFFLINE_TIMEOUT_MS 100U
#define MOTOR_REENABLE_PERIOD_MS 2000U
#define GAME_PROGRESS_BATTLE     4U
#define GAME_INFO_TIMEOUT_MS     1000U

static void SyncM3508State(motor_runtime_state_t *dst, struct motor_device *m, uint32_t now)
{
    int16_t pos = 0, vel = 0, current = 0;
    int8_t temp = 0;
    uint32_t dt;

    if (dst == NULL || m == NULL) return;

    m->get_status(m, "POS", &pos);
    m->get_status(m, "VEL", &vel);
    m->get_status(m, "CURRENT", &current);
    m->get_status(m, "TEMP", &temp);

    dst->pos = pos;
    dst->vel = vel;
    dst->current = current;
    dst->temp = temp;
    dst->last_rx_tick = m->last_rx_tick;

    dt = now - m->last_rx_tick;
    dst->online = (m->last_rx_tick != 0U && dt <= MOTOR_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static void SyncM2006State(motor_runtime_state_t *dst, struct motor_device *m, uint32_t now)
{
    int32_t pos_sum = 0;
    int16_t vel = 0, current = 0;
    int8_t temp = 0;
    uint32_t dt;

    if (dst == NULL || m == NULL) return;

    m->get_status(m, "POS_SUM", &pos_sum);
    m->get_status(m, "VEL", &vel);
    m->get_status(m, "CURRENT", &current);
    m->get_status(m, "TEMP", &temp);

    dst->pos = pos_sum;
    dst->vel = vel;
    dst->current = current;
    dst->temp = temp;
    dst->last_rx_tick = m->last_rx_tick;

    dt = now - m->last_rx_tick;
    dst->online = (m->last_rx_tick != 0U && dt <= MOTOR_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static void SyncGM6020State(motor_runtime_state_t *dst, struct motor_device *m, uint32_t now)
{
    float pos_rad = 0.0f;
    int16_t vel = 0;
    uint32_t dt;

    if (dst == NULL || m == NULL) return;

    m->get_status(m, "POS", &pos_rad);
    m->get_status(m, "VEL", &vel);

    dst->pos = (int32_t)(pos_rad * 10000.0f);
    dst->vel = vel;
    dst->current = 0;
    dst->temp = 0;
    dst->last_rx_tick = m->last_rx_tick;

    dt = now - m->last_rx_tick;
    dst->online = (m->last_rx_tick != 0U && dt <= MOTOR_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

static void SyncJ4310State(motor_runtime_state_t *dst, struct motor_device *m, uint32_t now)
{
    float pos = 0.0f, vel = 0.0f;
    int8_t temp_mos = 0;
    uint32_t dt;

    if (dst == NULL || m == NULL) return;

    m->get_status(m, "POS", &pos);
    m->get_status(m, "VEL", &vel);
    m->get_status(m, "TEMP_MOS", &temp_mos);

    dst->pos = (int32_t)(pos * 10000.0f);
    dst->vel = (int32_t)(vel * 100.0f);
    dst->current = 0;
    dst->temp = temp_mos;
    dst->last_rx_tick = m->last_rx_tick;

    dt = now - m->last_rx_tick;
    dst->online = (m->last_rx_tick != 0U && dt <= MOTOR_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief 电机任务执行函数
 * @note  优先级：High (1ms)
 */
void motor_task_func(void const * argument) {
    // 1. 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);
    Motor_System_PowerOn_Init();

    // 2. 获取所有电机句柄
    struct motor_device* pitch   = motor_get_device("J4310_PITCH");
    struct motor_device* yaw     = motor_get_device("GM6020_YAW");
    struct motor_device* shoot_l = motor_get_device("M3508_SHOOT_L");
    struct motor_device* shoot_r = motor_get_device("M3508_SHOOT_R");
    struct motor_device* stir_m  = motor_get_device("M2006_TRIGGER");
    struct motor_device* chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }

    // 3. 模式历史记录（用于边缘触发检测）
    static gimbal_mode_e  last_gimbal_mode  = GIMBAL_RELAX;
    static chassis_mode_e last_chassis_mode = CHASSIS_RELAX;
    static shoot_mode_e   last_shoot_mode   = SHOOT_STOP;
    static uint32_t last_reenable_tick = 0U;

    while (1) {
        uint32_t now = osKernelSysTick();

        /* --- A. 边缘触发：云台使能控制 --- */
        if (robot_ctrl.gimbal_mode != last_gimbal_mode) {
            if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                if(pitch) pitch->send_disable_cmd(pitch);
                if(yaw)   yaw->send_disable_cmd(yaw);
            } else {
                if(pitch) pitch->send_enable_cmd(pitch);
                if(yaw)   yaw->send_enable_cmd(yaw);
            }
            last_gimbal_mode = robot_ctrl.gimbal_mode;
        }

        /* --- B. 边缘触发：发射机构使能控制 --- */
        if (robot_ctrl.shoot_mode != last_shoot_mode) {
            if (robot_ctrl.shoot_mode == SHOOT_STOP) {
                if(shoot_l) shoot_l->send_disable_cmd(shoot_l);
                if(shoot_r) shoot_r->send_disable_cmd(shoot_r);
                if(stir_m)  stir_m->send_disable_cmd(stir_m);
            } else {
                if(shoot_l) shoot_l->send_enable_cmd(shoot_l);
                if(shoot_r) shoot_r->send_enable_cmd(shoot_r);
                if(stir_m)  stir_m->send_enable_cmd(stir_m);
            }
            last_shoot_mode = robot_ctrl.shoot_mode;
        }

        /* --- C. 边缘触发：底盘使能控制 --- */
        if (robot_ctrl.chassis_mode != last_chassis_mode) {
            for(int i=0; i<4; i++) {
                if(!chassis[i]) continue;
                if (robot_ctrl.chassis_mode == CHASSIS_RELAX)
                    chassis[i]->send_disable_cmd(chassis[i]);
                else
                    chassis[i]->send_enable_cmd(chassis[i]);
            }
            last_chassis_mode = robot_ctrl.chassis_mode;
        }

        if ((uint32_t)(now - last_reenable_tick) >= MOTOR_REENABLE_PERIOD_MS) {
            if (robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                if (pitch) pitch->send_enable_cmd(pitch);
                if (yaw)   yaw->send_enable_cmd(yaw);
            }

            if (robot_ctrl.shoot_mode != SHOOT_STOP) {
                if (stir_m)  stir_m->send_enable_cmd(stir_m);
            }

            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                for (int i = 0; i < 4; i++) {
                    if (chassis[i]) chassis[i]->send_enable_cmd(chassis[i]);
                }
            }

            last_reenable_tick = now;
        }

        /* --- D. 硬件指令下发 (每毫秒执行一次) --- */

        // 执行所有电机的计算回调（PID计算将 set_target 转为输出电流）
        Motor_All_Update();

        // 将控制电流发送至 CAN 总线
        DJI_Motor_Send_CAN1_Group(&hcan1);
        DJI_Motor_Send_CAN2_Group(&hcan2);

        // 达妙电机（Pitch轴）使用专用协议帧发送
        if(pitch) pitch->send_ctrl_cmd(pitch);

        /* --- E. 同步全部电机反馈到全局（含在线状态） --- */
        if (pitch)   SyncJ4310State(&robot_ctrl.motors_info.j4310_pitch, pitch, now);
        if (yaw)     SyncGM6020State(&robot_ctrl.motors_info.gm6020_yaw, yaw, now);
        if (stir_m)  SyncM2006State(&robot_ctrl.motors_info.m2006_trigger, stir_m, now);
        if (shoot_l) SyncM3508State(&robot_ctrl.motors_info.m3508_shoot_l, shoot_l, now);
        if (shoot_r) SyncM3508State(&robot_ctrl.motors_info.m3508_shoot_r, shoot_r, now);
        if (chassis[0]) SyncM3508State(&robot_ctrl.motors_info.m3508_chassis_1, chassis[0], now);
        if (chassis[1]) SyncM3508State(&robot_ctrl.motors_info.m3508_chassis_2, chassis[1], now);
        if (chassis[2]) SyncM3508State(&robot_ctrl.motors_info.m3508_chassis_3, chassis[2], now);
        if (chassis[3]) SyncM3508State(&robot_ctrl.motors_info.m3508_chassis_4, chassis[3], now);

        osDelay(1);
    }
}
