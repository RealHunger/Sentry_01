#include "gimbal_task.h"          // 云台任务头文件-本文件声明
#include "../Application/robot_global.h"  // 全局变量头文件-核心全局结构体、枚举定义
#include "../../Components/motor/motor.h" // 电机驱动头文件-电机设备句柄/接口函数
#include "math.h"                 // 数学库头文件-三角函数/绝对值/浮点运算
#include "stdlib.h"               // 标准库头文件-通用工具函数
#include "cmsis_os.h"             // RTOS系统头文件-系统滴答/延时/任务调度
#include "stdio.h"                // 标准输入输出-调试打印备用
#include "../../Components/remote/remote.h" // 遥控器驱动头文件-遥控数据解析

/***********************************************************************************************************************
* 宏定义-集中管理 【仅保留实际调用的有效宏定义，分区归类+详细注释，无冗余】
***********************************************************************************************************************/
// ===================== 云台控制-摇杆/鼠标 灵敏度&死区参数【云台核心必用】 =====================
#define RC_DEADZONE         10          // 遥控器摇杆死区：防止摇杆漂移产生无效信号
#define MOUSE_YAW_SENS      0.00005f    // 鼠标X轴-云台航向角 控制灵敏度
#define MOUSE_PIT_SENS      0.00005f    // 鼠标Y轴-云台俯仰角 控制灵敏度
#define RC_YAW_SENS         0.005f      // 遥控器摇杆-云台航向角 控制灵敏度
#define RC_PIT_SENS         0.005f      // 遥控器摇杆-云台俯仰角 控制灵敏度


// ===================== 云台核心限位 【物理机械硬限位，重中之重，严禁修改】 =====================
#define PITCH_UP_LIMIT      0.35f       // 云台俯仰角 向上最大限位 (弧度制) 防止云台撞上枪管/云台架
#define PITCH_DOWN_LIMIT    -0.45f      // 云台俯仰角 向下最大限位 (弧度制) 防止云台撞上底盘/发射机构

// ===================== 自瞄丢目标扫描参数 =====================
#define AUTO_SCAN_LOST_DELAY_MS 120U    // 丢目标持续超过该时间后开始扫描
#define AUTO_HOLD_ON_VALID_DROP_MS 1000U // valid 从1->0后先保持瞄准1秒
#define AUTO_SCAN_SPEED_RAD_S   2.4f    // 扫描角速度(rad/s)
#define AUTO_SCAN_PITCH_CENTER  0.0f    // 点头扫描中心角(rad)
#define AUTO_SCAN_PITCH_RANGE   0.30f   // 点头扫描半幅(rad)
#define AUTO_SCAN_PITCH_SPEED   2.4f    // 点头扫描角速度(rad/s)
#define AUTO_SCAN_PITCH_ACCEL   6.0f    // 点头扫描加速度(rad/s^2)，进入扫描后平滑升速
#define GIMBAL_TASK_DT_S        0.002f  // 本任务周期2ms

// ===================== 云台抗抖参数（底盘自转时优先稳态） =====================
#define YAW_ERR_DEADBAND_RAD    0.004f  // 小误差死区，抑制抖动
#define YAW_ERR_DEADBAND_MID    0.0050f // 中速档略放宽死区，减小旋转中的细碎抖动
#define YAW_ERR_DEADBAND_HIGH   0.0045f // 高速档略收死区，减小稳定后的残余偏差
#define YAW_DAMP_K_NORMAL       0.000f  // 常规档角速度阻尼
#define YAW_DAMP_K_MID          0.0012f // 中速档轻阻尼，压抖但不拖慢太多
#define YAW_DAMP_K_HIGH         0.0014f // 高速档轻阻尼，抑制过冲同时避免过分拖慢
#define YAW_FF_ALPHA_NORMAL     0.04f   // 常规档前馈一阶滤波系数
#define YAW_FF_ALPHA_MID        0.032f  // 中速档前馈滤波，兼顾响应和抑抖
#define YAW_FF_ALPHA_HIGH       0.030f  // 高速档前馈滤波略收，降低起步冲量
#define YAW_FF_LIMIT_NORMAL     200.0f  // 常规档前馈限幅
#define YAW_FF_LIMIT_MID        230.0f  // 中速档前馈限幅
#define YAW_FF_LIMIT_HIGH       280.0f  // 高转速档前馈限幅，适度增强但避免明显过冲
#define YAW_FF_GAIN_NORMAL      2.0f    // 常规档前馈比例
#define YAW_FF_GAIN_MID         2.15f   // 中速档前馈比例，补偿1.6档欠冲
#define YAW_FF_GAIN_HIGH        2.30f   // 高转速档前馈比例，补一点持续高速时的残余跟随误差
#define YAW_FF_SIGN             1.0f    // 前馈方向（若仍反向偏差，改为 -1.0f）
#define YAW_FF_STEP_MAX_NORMAL  1.6f    // 常规档前馈每周期最大变化量
#define YAW_FF_STEP_MAX_MID     2.0f    // 中速档前馈爬升速度
#define YAW_FF_STEP_MAX_HIGH    2.4f    // 高转速档前馈爬升速度，抑制刚开始的过冲

// 小积分只用于消除稳态微小偏差，避免大误差阶段过积分
#define YAW_I_GAIN_NORMAL       0.55f
#define YAW_I_GAIN_MID          0.72f
#define YAW_I_GAIN_HIGH         0.86f
#define YAW_I_LIMIT_NORMAL      0.10f
#define YAW_I_LIMIT_MID         0.14f
#define YAW_I_LIMIT_HIGH        0.18f
#define YAW_I_ACTIVE_ERR_RAD    0.24f

// ===================== 云台 yaw 高转速档专用参数 =====================
#define YAW_MID_SPEED_ENTER_RAD_S   70.0f
#define YAW_MID_SPEED_EXIT_RAD_S    55.0f
#define YAW_HIGH_SPEED_ENTER_RAD_S  120.0f
#define YAW_HIGH_SPEED_EXIT_RAD_S   90.0f
#define YAW_KP_P_NORMAL             420.0f
#define YAW_KP_P_MID                470.0f
#define YAW_KP_P_HIGH               520.0f
#define YAW_KD_P_NORMAL             1.45f
#define YAW_KD_P_MID                1.30f
#define YAW_KD_P_HIGH               1.20f
#define YAW_KP_V_NORMAL             200.0f
#define YAW_KP_V_MID                235.0f
#define YAW_KP_V_HIGH               270.0f
#define YAW_KP_V_ONLY_NORMAL        300.0f
#define YAW_KP_V_ONLY_MID           360.0f
#define YAW_KP_V_ONLY_HIGH          420.0f
#define YAW_OUT_MAX_NORMAL          25000.0f
#define YAW_OUT_MAX_MID             27000.0f
#define YAW_OUT_MAX_HIGH            30000.0f
#define YAW_V_LIMIT_NORMAL          320.0f
#define YAW_V_LIMIT_MID             400.0f
#define YAW_V_LIMIT_HIGH            480.0f

typedef enum {
    YAW_PROFILE_NORMAL = 0,
    YAW_PROFILE_MID_SPEED,
    YAW_PROFILE_HIGH_SPEED,
} yaw_profile_e;

static void yaw_apply_profile(const struct motor_device *yaw_m, yaw_profile_e profile)
{
    float kp_p;
    float kd_p;
    float kp_v;
    float kp_v_only;
    float out_max;
    float v_limit;

    if (yaw_m == NULL || yaw_m->set_para == NULL) {
        return;
    }

    if (profile == YAW_PROFILE_HIGH_SPEED) {
        kp_p = YAW_KP_P_HIGH;
        kd_p = YAW_KD_P_HIGH;
        kp_v = YAW_KP_V_HIGH;
        kp_v_only = YAW_KP_V_ONLY_HIGH;
        out_max = YAW_OUT_MAX_HIGH;
        v_limit = YAW_V_LIMIT_HIGH;
    } else if (profile == YAW_PROFILE_MID_SPEED) {
        kp_p = YAW_KP_P_MID;
        kd_p = YAW_KD_P_MID;
        kp_v = YAW_KP_V_MID;
        kp_v_only = YAW_KP_V_ONLY_MID;
        out_max = YAW_OUT_MAX_MID;
        v_limit = YAW_V_LIMIT_MID;
    } else {
        kp_p = YAW_KP_P_NORMAL;
        kd_p = YAW_KD_P_NORMAL;
        kp_v = YAW_KP_V_NORMAL;
        kp_v_only = YAW_KP_V_ONLY_NORMAL;
        out_max = YAW_OUT_MAX_NORMAL;
        v_limit = YAW_V_LIMIT_NORMAL;
    }

    yaw_m->set_para(yaw_m, "Kp_p", &kp_p);
    yaw_m->set_para(yaw_m, "Kd_p", &kd_p);
    yaw_m->set_para(yaw_m, "Kp_v", &kp_v);
    yaw_m->set_para(yaw_m, "Kp_v_only", &kp_v_only);
    yaw_m->set_para(yaw_m, "out_max", &out_max);
    yaw_m->set_para(yaw_m, "v_limit", &v_limit);
}

static float clampf(float v, float vmin, float vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

/***********************************************************************************************************************
* 函数名：Rad_Format
* 功  能：角度归一化处理，将任意弧度制角度限制在 [-π, π] 区间内
* 参  数：angle 待归一化的原始弧度角度
* 返回值：归一化后的合规弧度角度
* 说  明：解决云台360°旋转时角度值跳变问题，保证闭环控制的连续性，无突变抖动
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 函数名：gimbal_task_func
* 功  能：云台任务主函数 - 优先级最高的控制任务之一
* 职  责：1.云台模式切换(失能/手动/自瞄) 2.云台角度闭环控制 3.发射机构控制(摩擦轮+拨弹轮)
*         4.拨弹轮堵转逃逸保护 5.遥控器掉线急停保护 6.各状态指示灯反馈 7.自瞄数据解析与使用
* 参  数：argument RTOS任务形参，无实际使用
* 调度周期：2ms 高频率保证控制精度，云台控制核心要求
* 适配说明：已全部修改为【VT13遥控器专属】输入，无任何DT7相关代码
***********************************************************************************************************************/
void gimbal_task_func(void const * argument) {
    /**************************************** 【硬件外设初始化区】 ****************************************/
    // 获取所有电机设备句柄 - 绑定对应电机，通过句柄调用电机驱动接口
    const struct motor_device *yaw_m = motor_get_device("GM6020_YAW");    // 云台航向轴电机 GM6020
    const struct motor_device *pit_m = motor_get_device("J4310_PITCH");   // 云台俯仰轴电机 J4310

    /**************************************** 【静态状态变量区 - 防抖/状态机/计时专用，无冗余】 ****************************************/
    static uint8_t last_mode_toggle = 0;     // 云台模式切换按键 上一帧状态 - 按键防抖，防止误触
    static uint8_t last_shoot_on_toggle = 0;    // F 键上一帧状态（起转）
    static uint8_t last_shoot_off_toggle = 0;   // B 键上一帧状态（停转）
    static uint8_t is_initialized = 0;       // 云台初始化标志位 0-未初始化 1-已初始化 防止上电瞬间角度突变甩动
    static uint8_t was_auto_mode = 0;        // 上一帧是否处于自瞄模式
    static uint8_t auto_scan_active = 0;     // 自瞄丢目标扫描状态
    static int8_t auto_scan_pitch_dir = 1;   // 点头方向：1上抬，-1下压
    static float auto_scan_pitch_speed_cur = 0.0f; // 当前点头扫描速度，进入扫描时渐增
    static uint32_t last_target_seen_tick = 0U; // 最近一次检测到目标的时间戳
    static uint32_t valid_drop_hold_until_tick = 0U; // valid 下降沿后的保持截止时间
    static uint8_t last_target_valid = 0U;    // 上一帧目标有效状态，用于检测 1->0 下降沿
    static float yaw_ff_filtered = 0.0f;     // 底盘自转前馈滤波值
    static float yaw_i_term = 0.0f;          // yaw误差微积分项（仅消静差）
    static yaw_profile_e yaw_profile = YAW_PROFILE_NORMAL;
    float world_yaw_target = 0.0f;           // 云台世界坐标系 航向角目标值 (弧度)
    float world_pit_target = 0.0f;           // 云台世界坐标系 俯仰角目标值 (弧度)

    /**************************************** 【系统上电启动保护】 ****************************************/
    // 等待传感器就绪：陀螺仪/加速度计等传感器未就绪前，云台不动作，防止失控
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);  // 上电延时1s，等待所有外设/电机/传感器稳定，硬件防冲击

    /**************************************** 【云台任务主循环 - 死循环永不退出】 ****************************************/
    while (1) {
        uint32_t current_tick = osKernelSysTick();  // 获取当前系统滴答定时器值(ms)，用于所有计时逻辑

        /**************************************** 【最高优先级】VT13遥控器掉线全局急停保护 ****************************************/
        // 遥控器超时判定：使用有符号差值，避免并发更新导致无符号下溢误判
        int32_t rc_tick_diff = (int32_t)(current_tick - robot_ctrl.rc->vt13.last_update_tick);
        if (rc_tick_diff > 1000) {
            robot_ctrl.monitor.remote_online = 0;        // 置位遥控器离线标志位
            robot_ctrl.monitor.system_enabled = 0;       // 统一使能拉低，避免云台/底盘状态分叉
            robot_ctrl.monitor.plan_enabled = 0;
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;       // 云台强制进入失能模式，无动力
            robot_ctrl.shoot_mode = SHOOT_STOP;          // 发射机构强制停止，所有发射电机归零
            is_initialized = 0;                          // 云台初始化标志位清零，重连后重新初始化
            yaw_i_term = 0.0f;


            osDelay(100);
        }
        // ===================== VT13遥控器在线 正常工作逻辑 =====================
        else {
            robot_ctrl.monitor.remote_online = 1;  // 置位遥控器在线标志位

            /********************* 发射模式仲裁：S强制起转，C强制停转，N听键盘 *********************/
            uint8_t shoot_on_cmd = KEY_PRESSED(robot_ctrl.rc->vt13.key_vt13.v, KEY_VT13_F);
            uint8_t shoot_off_cmd = KEY_PRESSED(robot_ctrl.rc->vt13.key_vt13.v, KEY_VT13_B);
            uint8_t shoot_on_trigger = (shoot_on_cmd && !last_shoot_on_toggle);
            uint8_t shoot_off_trigger = (shoot_off_cmd && !last_shoot_off_toggle);
            uint8_t sw = robot_ctrl.rc->vt13.rc_vt13.sw;

            if (sw == RC_SW_S_VT13) {
                robot_ctrl.shoot_mode = SHOOT_READY;
            } else if (sw == RC_SW_C_VT13) {
                robot_ctrl.shoot_mode = SHOOT_STOP;
            } else { /* N档：由键盘控制 */
                if (shoot_on_trigger) {
                    robot_ctrl.shoot_mode = SHOOT_READY;
                }
                if (shoot_off_trigger) {
                    robot_ctrl.shoot_mode = SHOOT_STOP;
                }
            }

            last_shoot_on_toggle = shoot_on_cmd;
            last_shoot_off_toggle = shoot_off_cmd;

            // // VT13遥控器档位切换：S档(发射档) ↔ 其他档 切换，优先级与F键一致
            // if (robot_ctrl.rc->vt13.rc_vt13.sw != last_sw_state) {
            //     robot_ctrl.shoot_mode = (robot_ctrl.rc->vt13.rc_vt13.sw == RC_SW_S_VT13) ? SHOOT_READY : SHOOT_STOP;
            //     last_sw_state = robot_ctrl.rc->vt13.rc_vt13.sw;     // 更新档位上一帧状态，用于防抖
            // }
            /********************* 云台工作模式：读取统一使能状态，避免与底盘各自切换产生不同步 *********************/
            // 云台模式切换条件：VT13遥控器自定义左按键 或 鼠标右键 按下 (原先为 VT13 G 键)
            uint8_t mode_cmd = (robot_ctrl.rc->vt13.rc_vt13.custom_l) || (robot_ctrl.rc->vt13.mouse_vt13.press_r);
            uint8_t mode_trigger = (mode_cmd && !last_mode_toggle);     // 按键上升沿触发，防抖

            if (!robot_ctrl.monitor.system_enabled) {
                robot_ctrl.gimbal_mode = GIMBAL_RELAX;
                is_initialized = 0;
                yaw_ff_filtered = 0.0f;
                yaw_i_term = 0.0f;
            } else {
                if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                    robot_ctrl.gimbal_mode = GIMBAL_REMOTE;
                    is_initialized = 0;
                    yaw_i_term = 0.0f;
                }

                // 触发模式切换：手动 ↔ 自瞄 互切，仅在云台使能状态下有效
                if (mode_trigger) {
                    robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) ? GIMBAL_AUTO : GIMBAL_REMOTE;
                }
            }

            // 更新按键上一帧状态，完成防抖逻辑
            last_mode_toggle = mode_cmd;

            /**************************************** 云台角度闭环控制核心逻辑 ****************************************/
            if (robot_ctrl.gimbal_mode != GIMBAL_RELAX) {  // 云台非失能模式 → 使能，进入角度闭环控制
                // 云台首次使能初始化：将目标角度同步为当前实际角度，防止上电瞬间角度突变导致云台甩动
                if (is_initialized == 0) {
                    world_yaw_target = robot_ctrl.gimbal.yaw;
                    world_pit_target = robot_ctrl.gimbal.pitch;
                    is_initialized = 1;  // 置位初始化完成标志位，仅执行一次
                }

                /********************* 模式1：云台手动控制【VT13遥控器摇杆+鼠标 复合控制】 *********************/
                if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) {
                    // VT13遥控器摇杆值处理：死区过滤 + 归一化到[-1,1]区间，消除无效信号
                    float ry = (abs(robot_ctrl.rc->vt13.rc_vt13.ch[2]) > RC_DEADZONE) ? robot_ctrl.rc->vt13.rc_vt13.ch[2] / 660.0f : 0.0f;
                    float rx = (abs(robot_ctrl.rc->vt13.rc_vt13.ch[3]) > RC_DEADZONE) ? robot_ctrl.rc->vt13.rc_vt13.ch[3] / 660.0f : 0.0f;
                    // VT13鼠标值处理：直接乘以灵敏度系数，转为角度增量
                    float mouse_x = (float)robot_ctrl.rc->vt13.mouse_vt13.x * MOUSE_YAW_SENS;
                    float mouse_y = (float)robot_ctrl.rc->vt13.mouse_vt13.y * MOUSE_PIT_SENS;

                    // 计算云台目标角度：摇杆控制量 + 鼠标控制量 叠加
                    world_pit_target -= (ry * RC_PIT_SENS) + mouse_y;
                    world_yaw_target -= (rx * RC_YAW_SENS) + mouse_x;

                    // 俯仰角目标值软件限位 【第一道防护】严格限制在机械限位内
                    if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                    if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                }

                /********************* 模式2：云台自瞄控制【核心优化】解析全局自瞄数据，视觉闭环 *********************/
                else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO) {
                    if (!was_auto_mode) {
                        auto_scan_active = 0U;
                        auto_scan_pitch_dir = 1;
                        auto_scan_pitch_speed_cur = 0.0f;
                        last_target_seen_tick = current_tick;
                        valid_drop_hold_until_tick = 0U;
                        last_target_valid = 0U;
                    }

                    uint8_t target_valid_now = (robot_ctrl.monitor.vision_online == 1U &&
                                                isfinite(robot_ctrl.target_info.aim_target_yaw) &&
                                                isfinite(robot_ctrl.target_info.aim_target_pitch)) ? 1U : 0U;

                    // com_task 已完成视觉数据解析，这里只消费 target_info
                    if (target_valid_now) {
                        // 直接赋值视觉解算后的目标角度，云台跟随目标
                        world_yaw_target = robot_ctrl.target_info.aim_target_yaw;
                        world_pit_target = robot_ctrl.target_info.aim_target_pitch;
                        last_target_seen_tick = current_tick;
                        auto_scan_active = 0U;
                        auto_scan_pitch_dir = 1;
                        auto_scan_pitch_speed_cur = 0.0f;
                        // 自瞄模式同样做俯仰角限位，防止视觉数据异常超限
                        if(world_pit_target > PITCH_UP_LIMIT)  world_pit_target = PITCH_UP_LIMIT;
                        if(world_pit_target < PITCH_DOWN_LIMIT)world_pit_target = PITCH_DOWN_LIMIT;
                        valid_drop_hold_until_tick = 0U;

                    } else {
                        // 仅在 valid 1->0 的下降沿触发保持窗口
                        if (last_target_valid == 1U) {
                            valid_drop_hold_until_tick = current_tick + AUTO_HOLD_ON_VALID_DROP_MS;
                            last_target_seen_tick = current_tick;
                            auto_scan_active = 0U;
                            auto_scan_pitch_dir = 1;
                            auto_scan_pitch_speed_cur = 0.0f;
                        }

                        // 保持阶段：冻结当前目标角，不进入扫描
                        if (valid_drop_hold_until_tick != 0U &&
                            (int32_t)(current_tick - valid_drop_hold_until_tick) < 0) {
                            // keep aiming for 1s after valid drop
                        } else {
                            valid_drop_hold_until_tick = 0U;

                            if (!auto_scan_active) {
                                if ((uint32_t)(current_tick - last_target_seen_tick) >= AUTO_SCAN_LOST_DELAY_MS) {
                                    auto_scan_active = 1U;
                                    world_pit_target = AUTO_SCAN_PITCH_CENTER;
                                    auto_scan_pitch_dir = 1;
                                    auto_scan_pitch_speed_cur = 0.0f;
                                }
                            }

                            if (auto_scan_active) {
                                // 连续单方向旋转，转满360度后由Rad_Format归一化。
                                float step = AUTO_SCAN_SPEED_RAD_S * GIMBAL_TASK_DT_S;
                                world_yaw_target = Rad_Format(world_yaw_target + step);

                                // Pitch 扫描速度平滑上升，避免进入扫描瞬间突变。
                                auto_scan_pitch_speed_cur += AUTO_SCAN_PITCH_ACCEL * GIMBAL_TASK_DT_S;
                                if (auto_scan_pitch_speed_cur > AUTO_SCAN_PITCH_SPEED) {
                                    auto_scan_pitch_speed_cur = AUTO_SCAN_PITCH_SPEED;
                                }
                                float pit_step = auto_scan_pitch_speed_cur * GIMBAL_TASK_DT_S * (float)auto_scan_pitch_dir;
                                world_pit_target += pit_step;

                                if (world_pit_target >= (AUTO_SCAN_PITCH_CENTER + AUTO_SCAN_PITCH_RANGE)) {
                                    world_pit_target = AUTO_SCAN_PITCH_CENTER + AUTO_SCAN_PITCH_RANGE;
                                    auto_scan_pitch_dir = -1;
                                } else if (world_pit_target <= (AUTO_SCAN_PITCH_CENTER - AUTO_SCAN_PITCH_RANGE)) {
                                    world_pit_target = AUTO_SCAN_PITCH_CENTER - AUTO_SCAN_PITCH_RANGE;
                                    auto_scan_pitch_dir = 1;
                                }

                                if (world_pit_target > PITCH_UP_LIMIT) world_pit_target = PITCH_UP_LIMIT;
                                if (world_pit_target < PITCH_DOWN_LIMIT) world_pit_target = PITCH_DOWN_LIMIT;
                            }
                        }
                    }

                    last_target_valid = target_valid_now;
                }

                was_auto_mode = (robot_ctrl.gimbal_mode == GIMBAL_AUTO) ? 1U : 0U;

                /********************* 云台角度闭环输出 + 双重限位保护 【最终防护】 *********************/
                float cur_yaw, cur_pit;
                float yaw_ff_gain;
                float yaw_ff_limit;
                float yaw_ff_alpha;
                float yaw_ff_step_max;
                float yaw_deadband;
                float yaw_damp_k;
                float yaw_i_gain;
                float yaw_i_limit;
                float abs_yaw_speed = fabsf(robot_ctrl.chassis.yaw_speed);
                yaw_m->get_status(yaw_m, "POS", &cur_yaw);  // 获取航向轴电机 当前实际角度
                pit_m->get_status(pit_m, "POS", &cur_pit);  // 获取俯仰轴电机 当前实际角度

                if ((yaw_profile != YAW_PROFILE_HIGH_SPEED) && (abs_yaw_speed >= YAW_HIGH_SPEED_ENTER_RAD_S)) {
                    yaw_profile = YAW_PROFILE_HIGH_SPEED;
                    yaw_apply_profile(yaw_m, yaw_profile);
                } else if ((yaw_profile == YAW_PROFILE_HIGH_SPEED) && (abs_yaw_speed <= YAW_HIGH_SPEED_EXIT_RAD_S)) {
                    yaw_profile = (abs_yaw_speed >= YAW_MID_SPEED_ENTER_RAD_S) ? YAW_PROFILE_MID_SPEED : YAW_PROFILE_NORMAL;
                    yaw_apply_profile(yaw_m, yaw_profile);
                } else if ((yaw_profile == YAW_PROFILE_NORMAL) && (abs_yaw_speed >= YAW_MID_SPEED_ENTER_RAD_S)) {
                    yaw_profile = YAW_PROFILE_MID_SPEED;
                    yaw_apply_profile(yaw_m, yaw_profile);
                } else if ((yaw_profile == YAW_PROFILE_MID_SPEED) && (abs_yaw_speed <= YAW_MID_SPEED_EXIT_RAD_S)) {
                    yaw_profile = YAW_PROFILE_NORMAL;
                    yaw_apply_profile(yaw_m, yaw_profile);
                }

                if (yaw_profile == YAW_PROFILE_HIGH_SPEED) {
                    yaw_ff_gain = YAW_FF_GAIN_HIGH;
                    yaw_ff_limit = YAW_FF_LIMIT_HIGH;
                    yaw_ff_alpha = YAW_FF_ALPHA_HIGH;
                    yaw_ff_step_max = YAW_FF_STEP_MAX_HIGH;
                    yaw_deadband = YAW_ERR_DEADBAND_HIGH;
                    yaw_damp_k = YAW_DAMP_K_HIGH;
                    yaw_i_gain = YAW_I_GAIN_HIGH;
                    yaw_i_limit = YAW_I_LIMIT_HIGH;
                } else if (yaw_profile == YAW_PROFILE_MID_SPEED) {
                    yaw_ff_gain = YAW_FF_GAIN_MID;
                    yaw_ff_limit = YAW_FF_LIMIT_MID;
                    yaw_ff_alpha = YAW_FF_ALPHA_MID;
                    yaw_ff_step_max = YAW_FF_STEP_MAX_MID;
                    yaw_deadband = YAW_ERR_DEADBAND_MID;
                    yaw_damp_k = YAW_DAMP_K_MID;
                    yaw_i_gain = YAW_I_GAIN_MID;
                    yaw_i_limit = YAW_I_LIMIT_MID;
                } else {
                    yaw_ff_gain = YAW_FF_GAIN_NORMAL;
                    yaw_ff_limit = YAW_FF_LIMIT_NORMAL;
                    yaw_ff_alpha = YAW_FF_ALPHA_NORMAL;
                    yaw_ff_step_max = YAW_FF_STEP_MAX_NORMAL;
                    yaw_deadband = YAW_ERR_DEADBAND_RAD;
                    yaw_damp_k = YAW_DAMP_K_NORMAL;
                    yaw_i_gain = YAW_I_GAIN_NORMAL;
                    yaw_i_limit = YAW_I_LIMIT_NORMAL;
                }

                // 云台闭环控制算法：航向角带底盘速度前馈补偿，俯仰角直接位置闭环，保证跟随精度
                float yaw_err = Rad_Format(world_yaw_target - robot_ctrl.gimbal.yaw);
                if (fabsf(yaw_err) < yaw_deadband) {
                    yaw_err = 0.0f;
                }

                float yaw_ff_raw = clampf((YAW_FF_SIGN * yaw_ff_gain) * robot_ctrl.chassis.yaw_speed,
                                          -yaw_ff_limit, yaw_ff_limit);

                // 先限斜率再滤波，减少自转起步时前馈瞬态过冲
                float ff_delta = yaw_ff_raw - yaw_ff_filtered;
                if (ff_delta > yaw_ff_step_max) ff_delta = yaw_ff_step_max;
                if (ff_delta < -yaw_ff_step_max) ff_delta = -yaw_ff_step_max;
                yaw_ff_filtered += ff_delta;
                yaw_ff_filtered += yaw_ff_alpha * (yaw_ff_raw - yaw_ff_filtered);

                // 仅在小误差区启用微积分，专门吃掉稳态残余误差
                if (fabsf(yaw_err) < YAW_I_ACTIVE_ERR_RAD) {
                    yaw_i_term += yaw_i_gain * yaw_err * GIMBAL_TASK_DT_S;
                    yaw_i_term = clampf(yaw_i_term, -yaw_i_limit, yaw_i_limit);
                } else {
                    yaw_i_term *= 0.995f;
                }

                float yaw_out = cur_yaw + yaw_err + yaw_i_term - (yaw_damp_k * robot_ctrl.gimbal.yaw_v);
                float pit_out = cur_pit - (world_pit_target - robot_ctrl.gimbal.pitch);

                // 俯仰角输出值二次限位 【第二道防护，终极防护】防止任何情况超限
                if (pit_out > PITCH_UP_LIMIT) pit_out = PITCH_UP_LIMIT;
                if (pit_out < PITCH_DOWN_LIMIT) pit_out = PITCH_DOWN_LIMIT;

                // 下发目标角度到电机闭环控制器，电机执行跟随
                yaw_m->set_target(yaw_m, 2, yaw_out, yaw_ff_filtered); // 航向角前馈经过滤波限幅，减少自转抖动
                //yaw_m->set_target(yaw_m, 2, yaw_out, 36.0f);

                pit_m->set_target(pit_m, 1, pit_out);
            }
            /********************* 模式3：云台失能模式 *********************/
            else if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                was_auto_mode = 0U;
                auto_scan_pitch_dir = 1;
                auto_scan_pitch_speed_cur = 0.0f;
                yaw_ff_filtered = 0.0f;
                yaw_i_term = 0.0f;
            }

            //调试用：
            //int16_t yaw_speed;
            //yaw_m->get_status(yaw_m, "VEL", &yaw_speed);
            //Uart->Print(Uart, "%d,%f\r\n", yaw_speed, robot_ctrl.chassis.yaw_speed); // 调试打印航向角目标值，单位：mrad
            //Uart->Print(Uart, "%f,%f\r\n", robot_ctrl.gimbal.yaw, world_yaw_target); // 调试打印航向角目标值，单位：mrad

        }
        osDelay(2);  // 云台任务调度周期 2ms，固定频率保证控制精度
    }
}

