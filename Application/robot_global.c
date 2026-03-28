#include "robot_global.h"
#include "string.h"

/* 实例化全局控制变量 */
robot_ctrl_info_t robot_ctrl;

/**
 * @brief 全局控制变量初始化
 * @note  在系统上电时调用，确保所有模式初始为安全状态
 */
void Robot_Global_Init(void) {
    // 结构体整体清零 (将所有浮点数置0，指针置空，新增的target_info也会被清零)
    memset(&robot_ctrl, 0, sizeof(robot_ctrl_info_t));

    // 初始模式设置
    robot_ctrl.gimbal_mode  = GIMBAL_RELAX;
    robot_ctrl.chassis_mode = CHASSIS_RELAX;
    robot_ctrl.shoot_mode   = SHOOT_STOP;

    RC_Init();

    // 关联遥控器句柄 (需要确保 RC_get_handle 返回的是包含遥控器数据的静态指针)
    robot_ctrl.rc = RC_Get_Handle();

    // 状态标志显式初始化
    robot_ctrl.monitor.sensor_ready = 0;
    robot_ctrl.monitor.remote_online = 0;
    robot_ctrl.monitor.system_enabled = 1;
    robot_ctrl.monitor.plan_enabled = 1;
}
