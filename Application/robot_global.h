#ifndef ROBOT_GLOBAL_H
#define ROBOT_GLOBAL_H

#include "struct_typedef.h"
#include "stdint.h"
#include "../Components/remote/remote.h"
#include "../../Application/auto_ctrl.h"  //【新增】引入自瞄头文件，支持target_info_t结构体

/* --- 模式枚举定义 --- */

typedef enum {
    GIMBAL_RELAX = 0,    // 失能状态，电机不出力
    GIMBAL_REMOTE,       // 遥控器手动模式（基于IMU控制）
    GIMBAL_AUTO,         // 视觉自瞄模式
} gimbal_mode_e;

typedef enum {
    CHASSIS_RELAX = 0,   // 失能状态
    CHASSIS_FOLLOW,      // 跟随模式（以云台朝向为正前方）
} chassis_mode_e;

typedef enum {
    SHOOT_STOP = 0,      // 停止发射
    SHOOT_READY,         // 摩擦轮起旋
} shoot_mode_e;


/* --- 电机反馈与在线状态缓存 --- */
typedef struct {
    int32_t pos;
    int32_t vel;
    int32_t current;
    int32_t temp;
    uint32_t last_rx_tick;
    uint8_t online;
} motor_runtime_state_t;

typedef struct {
    motor_runtime_state_t j4310_pitch;
    motor_runtime_state_t gm6020_yaw;
    motor_runtime_state_t m2006_trigger;
    motor_runtime_state_t m3508_shoot_l;
    motor_runtime_state_t m3508_shoot_r;
    motor_runtime_state_t m3508_chassis_1;
    motor_runtime_state_t m3508_chassis_2;
    motor_runtime_state_t m3508_chassis_3;
    motor_runtime_state_t m3508_chassis_4;
} motors_info_t;

/* --- CAN2 对端遥测缓存（来自 0x301/0x302） --- */
typedef struct {
    /* 0x301 */
    uint8_t robot_id;
    uint8_t game_progress;
    uint16_t stage_remain_time;
    uint16_t current_HP;
    int16_t capacity_voltage;

    /* 0x302 */
    uint16_t shooter_17mm_barrel_heat;
    uint8_t armor_id;
    uint8_t center_bonus_state;
    uint8_t rfid_supply19;
    uint8_t rfid_center23;

    /* 状态 */
    uint32_t last_tick_301;
    uint32_t last_tick_302;
    uint8_t online_301;
    uint8_t online_302;
} game_info;

/* --- 核心控制结构体 --- */

typedef struct {
    // 1. 系统当前运行模式
    gimbal_mode_e  gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e   shoot_mode;

    // 2. 云台姿态反馈数据 (由 Sensor Task 更新)
    struct {
        fp32 q[4];       // 当前姿态四元数 [w,x,y,z]
        fp32 yaw;        // 当前航向角 (度)
        fp32 pitch;      // 当前俯仰角 (度)
        fp32 roll;       // 当前横滚角 (度)
        fp32 yaw_v;      // 航向角速度 (度/s)
        fp32 pitch_v;    // 俯仰角速度 (度/s)
    } gimbal;

    // 3. 底盘运动状态 (由 Chassis Task 更新)
    struct {
        fp32 yaw_speed;      // 云台相对于底盘的机械夹角 (由编码器转化)
        fp32 cmd_vx;         // 上位机规划x速度（正方向：底盘前方）
        fp32 cmd_vy;         // 上位机规划y速度（正方向：底盘左侧）
    } chassis;

    // 4. 系统监控与异常处理
    struct {
        uint8_t  sensor_ready;   // 传感器校准完成标志
        uint8_t  remote_online;  // 遥控器在线标志
        uint8_t  vision_online;  // 视觉系统在线标志
        uint8_t  system_enabled; // 统一使能状态（1=云台/底盘使能，0=全部失能）
        uint8_t  plan_enabled;   // 底盘路径规划输入开关（custom_r上升沿切换）
        uint32_t auto_aim_startup_block_ms; // 开赛后延迟启用自瞄时长(ms)
    } monitor;

    // 5. 输入引用指针
    const RC_ctrl_t *rc;         // 遥控器原始数据引用

    // ==========【新增核心】自瞄视觉数据 - 全局共享 ==========
    target_info_t target_info;   // 上位机下发的自瞄数据(valid,shoot,yaw,pitch)

    // 6. CAN2 对端遥测信息（比赛信息/电容信息）
    game_info game_info;

    // 7. 本机所有电机反馈与在线状态
    motors_info_t motors_info;

} robot_ctrl_info_t;

/* --- 全局变量声明 --- */
extern robot_ctrl_info_t robot_ctrl;

/* --- 核心工具函数 --- */
void Robot_Global_Init(void);

#endif