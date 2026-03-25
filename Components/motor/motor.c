//
// Created by 14717 on 2025/8/27.
//

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "motor.h"
#include "math.h"
#include "cmsis_os.h"
#include "../../Bsp/LED/bsp_LED.h"
#include "../../Application/robot_global.h"

#define pi (fp32)M_PI

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**********************************************************************************************************************/
/* 达妙电机——MIT控制 */

struct DM_MIT_data {
    //控制量
    float _p_des;
    float _v_des;
    float _t_ff;

    //参数
    float _kp;
    float _kd;

    //反馈量
    int8_t ERR;  //电机错误状态反馈
    int16_t POS;  //当前电机位置反馈
    int16_t VEL;  //当前电机转速反馈
    int16_t T;  //当前电机转矩反馈
    int8_t TEMP_MOS;  //电机MOS温度反馈
    int8_t TEMP_Rotor;  //电机线圈温度反馈

    //控制幅值
    float P_MAX;
    float V_MAX;
    float T_MAX;

    // 非阻塞恢复控制时间戳（用于按反馈帧状态自动清错/重使能）
    uint32_t last_clear_cmd_tick;
    uint32_t last_enable_cmd_tick;
    uint8_t enable_requested;
};

#define DM_ERR_DISABLED      0x0
#define DM_ERR_ENABLED       0x1
#define DM_ERR_FAULT_MIN     0x8
#define DM_ERR_FAULT_MAX     0xE

#define DM_CLEAR_RETRY_MS    50U
#define DM_ENABLE_RETRY_MS   20U

static void DM_send_special_cmd(struct motor_device *motor, uint8_t cmd)
{
    if (motor == NULL || motor->motor_data == NULL || motor->motor_can_handle == NULL) return;

    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;

    for (int i = 0; i < 7; ++i) {
        tx_data[i] = 0xFF;
    }
    tx_data[7] = cmd;

    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

void DM_MIT_init(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL) return;

    /* 确保 motor->motor_data 已分配（调用者负责） */
    if (motor->motor_data == NULL) return;

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    /* 清零结构 */
    memset(d, 0, sizeof(*d));

    /* 存 ID（保留 11 位安全掩码） */
    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);

    /* 如果传入 CAN 句柄，复制到 device 中以便后续使用；否则 caller 应保证 motor->motor_can_handle 有效 */
    if (hcan != NULL) {
        motor->motor_can_handle = hcan;
    }


    /* 处理可变参数 */
    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);
        if (para_num >= 1) {
            const double v = va_arg(ap, double); d->_kp = (float)v;
        }
        if (para_num >= 2) {
            const double v = va_arg(ap, double); d->_kd = (float)v;
        }
        if (para_num >= 3) {
            const double v = va_arg(ap, double); d->P_MAX = (float)v;
        }
        if (para_num >= 4) {
            const double v = va_arg(ap, double); d->V_MAX = (float)v;
        }
        if (para_num >= 5) {
            const double v = va_arg(ap, double); d->T_MAX = (float)v;
        }

        va_end(ap);
    }

    {
        uint32_t send_mail_box;
        CAN_TxHeaderTypeDef enable_tx_message;
        uint8_t enable_can_send_data[8];

        memset(&enable_tx_message, 0, sizeof(enable_tx_message));
        enable_tx_message.StdId = (uint32_t)(motor->motor_id & 0x7FFU);
        enable_tx_message.IDE = CAN_ID_STD;
        enable_tx_message.RTR = CAN_RTR_DATA;
        enable_tx_message.DLC = 0x08;

        for (int i = 0; i < 7; ++i) enable_can_send_data[i] = 0xFF;
        enable_can_send_data[7] = 0xFC; // 初始化使能帧标识

        HAL_CAN_AddTxMessage(motor->motor_can_handle, &enable_tx_message, enable_can_send_data, &send_mail_box);
    }
}

/* 解析 CAN 8 字节反馈帧：
   D[0]: ID(低4位) | ERR(高4位)
   D[1]: POS[15:8]
   D[2]: POS[7:0]
   D[3]: VEL[11:4]
   D[4]: VEL[3:0] | T[11:8]
   D[5]: T[7:0]
   D[6]: T_MOS
   D[7]: T_Rotor
 */
void DM_get_measure(const struct motor_device *motor, const uint8_t *data)
{
    if (motor == NULL || motor->motor_data == NULL || data == NULL) {
        return;
    }

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    /* ERR：高 4 位 */
    d->ERR = (int8_t)((data[0] >> 4) & 0x0F);

    /* ID：低 4 位，不需要解析 */
    /* uint8_t id = data[0] & 0x0F; */

    /* POS：16 位有符号 */
    d->POS = (int16_t)((uint16_t)data[1] << 8 | (uint16_t)data[2]);

    /* VEL：12 位有符号，拼接后右移 4 位 */
    {
        const uint16_t vel_raw = (uint16_t)data[3] << 8 | (uint16_t)data[4];
        /* 12 位有符号扩展 */
        int16_t vel12 = (int16_t)(vel_raw >> 4);
        /* 如果最高位（位11）为 1，需要符号扩展到 16 位 */
        if (vel12 & (1 << 11)) {
            vel12 |= 0xF000; /* 保留上位 4 位为 1 */
        }
        d->VEL = vel12;
    }

    /* T：12 位有符号，高 4 位在 data[4] 的低 4 位，低 8 位在 data[5] */
    {
        uint16_t t_raw = ((uint16_t)(data[4] & 0x0F) << 8) | (uint16_t)data[5];
        int16_t t12 = (int16_t)t_raw;
        if (t12 & (1 << 11)) {
            t12 |= 0xF000;
        }
        d->T = t12;
    }

    /* T_MOS：Data[6]（单位：℃）*/
    d->TEMP_MOS = (int8_t)data[6];

    /* T_Rotor：Data[7]（单位：℃）*/
    d->TEMP_Rotor = (int8_t)data[7];
}

void DM_update(struct motor_device *motor)
{
    // 此处可添加状态更新逻辑（如滤波等），目前为空
}

/* MIT 模式控制帧打包并发送（原DM_MIT_enable，重命名为更贴合功能的名称） */
void DM_MIT_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) return;

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;
    uint32_t now = osKernelSysTick();

    // 软件失能锁存：上层调用 disable 后，不再自动清错/重使能
    if (d->enable_requested == 0U) {
        return;
    }

    // 基于反馈 ERR 状态做非阻塞恢复：故障先清错，失能先重使能，只有使能态才下发 MIT 控制
    if (d->ERR >= DM_ERR_FAULT_MIN && d->ERR <= DM_ERR_FAULT_MAX) {
        if ((uint32_t)(now - d->last_clear_cmd_tick) >= DM_CLEAR_RETRY_MS) {
            DM_send_special_cmd(motor, 0xFB); // 清除错误
            d->last_clear_cmd_tick = now;
        }
        return;
    }

    if (d->ERR == DM_ERR_DISABLED) {
        if ((uint32_t)(now - d->last_enable_cmd_tick) >= DM_ENABLE_RETRY_MS) {
            DM_send_special_cmd(motor, 0xFC); // 使能
            d->last_enable_cmd_tick = now;
        }
        return;
    }

    /* p_des: 16-bit unsigned */
    uint16_t pdes = (uint16_t)((d->_p_des + d->P_MAX) / (d->P_MAX * 2.0f) * 65535.0f);
    if (pdes < 0) pdes = 0;
    if (pdes > 0xFFFF) pdes = 0xFFFF;

    /* v_des: 12-bit unsigned */
    uint16_t vdes = (int16_t)((d->_v_des + d->V_MAX) / (d->V_MAX * 2.0f) * 4095.0f);
    if (vdes < 0) vdes = 0;
    if (vdes > 0x0FFF) vdes = 0x0FFF;

    /* Kp: 12-bit unsigned 文档范围[0,500] */
    uint16_t kp = (int16_t)(d->_kp / 500.0f * 4095.0f);
    if (kp < 0) kp = 0;
    if (kp > 0x0FFF) kp = 0x0FFF;

    /* Kd: 12-bit unsigned 文档范围[0,5] */
    uint16_t kd = (int16_t)(d->_kd / 5.0f * 4095.0f);
    if (kd < 0) kd = 0;
    if (kd > 0x0FFF) kd = 0x0FFF;

    /* t_ff: 12-bit unsigned (0..4095) */
    uint16_t t_ff = (int16_t)((d->_t_ff + d->T_MAX) / (d->T_MAX * 2.0f) * 4095.0f);
    if (t_ff < 0) t_ff = 0;
    if (t_ff > 0x0FFF) t_ff = 0x0FFF;

    /* 组帧 */
    CAN_TxHeaderTypeDef tx_msg;
    uint8_t tx_data[8];
    uint32_t send_mail_box = 0;

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.StdId = (uint32_t)(motor->motor_id & 0x7FFU); /* 帧 ID 等于设定 CAN ID */
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 8;

    tx_data[0] = (uint8_t)(pdes >> 8);
    tx_data[1] = (uint8_t)(pdes & 0xFF);

    /* v_des 12 位：高 8 位放在 D[2]（即 vdes >> 4），低 4 位放在 D[3] 高 4 位 */
    tx_data[2] = (uint8_t)(vdes >> 4);
    tx_data[3] = (uint8_t)((vdes & 0x0F) << 4);

    /* Kp 12 位：高 4 位拼到 D[3] 低 4 位，低 8 位放 D[4] */
    tx_data[3] |= (uint8_t)((kp >> 8) & 0x0F);
    tx_data[4] = (uint8_t)(kp & 0xFF);

    /* Kd 12 位：高 8 位放 D[5]（即 Kd >> 4），低 4 位放 D[6] 高 4 位 */
    tx_data[5] = (uint8_t)(kd >> 4);
    tx_data[6] = (uint8_t)((kd & 0x0F) << 4);

    /* t_ff 12 位：高 4 位拼到 D[6] 低 4 位，低 8 位放 D[7] */
    tx_data[6] |= (uint8_t)((t_ff >> 8) & 0x0F);
    tx_data[7] = (uint8_t)(t_ff & 0xFF);

    /* 发送 */
    HAL_CAN_AddTxMessage(motor->motor_can_handle, &tx_msg, tx_data, &send_mail_box);
}

/* 电机失能指令帧发送（原DM_disable */
void DM_MIT_send_disable_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;
    d->enable_requested = 0U;
    DM_send_special_cmd(motor, 0xFD);
}

// 使能帧的发送函数
void DM_MIT_send_enable_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    d->enable_requested = 1U;

    // 先发清错再发使能（非阻塞），后续由 send_ctrl_cmd 基于 ERR 自动重试
    DM_send_special_cmd(motor, 0xFB);
    DM_send_special_cmd(motor, 0xFC);
    d->last_clear_cmd_tick = osKernelSysTick();
    d->last_enable_cmd_tick = d->last_clear_cmd_tick;
}

void DM_MIT_set_target(const struct motor_device *motor, const int para_num, ...) {
    if (motor == NULL || motor->motor_data == NULL) return;

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    va_list ap;
    va_start(ap, para_num);

    if (para_num >= 1) {
        const double v = va_arg(ap, double); d->_p_des = (float)v;
    }
    if (para_num >= 2) {
        const double v = va_arg(ap, double); d->_v_des = (float)v;
    }
    if (para_num >= 3) {
        const double v = va_arg(ap, double); d->_t_ff = (float)v;
    }

    va_end(ap);
}

void DM_MIT_get_status(const struct motor_device *motor, const char* which_status, void* status_data) {
    if (motor == NULL || motor->motor_data == NULL || which_status == NULL || status_data == NULL) return;

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    if (strcmp(which_status, "ERR") == 0) {
        *(int8_t *)status_data = d->ERR;
    } else if (strcmp(which_status, "POS") == 0) {
        *(float *) status_data = (float) d->POS / 65535.0f * 2 * d->P_MAX;
        if (*(float *) status_data > 0.0f) {
            *(float *) status_data -= d->P_MAX;
        } else {
            *(float *) status_data += d->P_MAX;
        }
    } else if (strcmp(which_status, "VEL") == 0) {
        *(float *)status_data = (float)d->VEL / 4096.0f * 2 * d->V_MAX;
        if (*(float *)status_data > 0.0f) {
            *(float *)status_data -= d->V_MAX;
        }else if (*(float *)status_data <= 0.0f) {
            *(float *)status_data += d->V_MAX;
        }
    } else if (strcmp(which_status, "T") == 0) {
        *(float *)status_data = (float)d->T / 4096.0f * 2 * d->T_MAX;
        if (*(float *)status_data > 0.0f) {
            *(float *)status_data -= d->T_MAX;
        }else if (*(float *)status_data <= 0.0f) {
            *(float *)status_data += d->T_MAX;
        }
    } else if (strcmp(which_status, "TEMP_MOS") == 0) {
        *(int8_t *)status_data = d->TEMP_MOS;
    } else if (strcmp(which_status, "TEMP_Rotor") == 0) {
        *(int8_t *)status_data = d->TEMP_Rotor;
    } else if (strcmp(which_status, "P_MAX") == 0) {
        *(float *)status_data = d->P_MAX;
    } else if (strcmp(which_status, "V_MAX") == 0) {
        *(float *)status_data = d->V_MAX;
    } else if (strcmp(which_status, "T_MAX") == 0) {
        *(float *)status_data = d->T_MAX;
    } else if (strcmp(which_status, "Kp") == 0) {
        *(float *)status_data = d->_kp;
    } else if (strcmp(which_status, "Kd") == 0) {
        *(float *)status_data = d->_kd;
    } else if (strcmp(which_status, "p_des") == 0) {
        *(float *)status_data = d->_p_des;
    } else if (strcmp(which_status, "v_des") == 0) {
        *(float *)status_data = d->_v_des;
    } else if (strcmp(which_status, "tff") == 0) {
        *(float *)status_data = d->_t_ff;
    }

}

void DM_MIT_set_para(const struct motor_device *motor, const char* which_para, void* para_data) {
    if (motor == NULL || motor->motor_data == NULL || which_para == NULL || para_data == NULL) return;

    struct DM_MIT_data *d = (struct DM_MIT_data *)motor->motor_data;

    if (strcmp(which_para, "Kp") == 0) {
        d->_kp = *(float *)para_data;
        if (d->_kp < 0.0f) d->_kp = 0.0f;
        if (d->_kp > 500.0f) d->_kp = 500.0f;
    } else if (strcmp(which_para, "Kd") == 0) {
        d->_kd = *(float *)para_data;
        if (d->_kd < 0.0f) d->_kd = 0.0f;
        if (d->_kd > 5.0f) d->_kd = 5.0f;
    } else if (strcmp(which_para, "P_MAX") == 0) {
        d->P_MAX = *(float *)para_data;
    } else if (strcmp(which_para, "V_MAX") == 0) {
        d->V_MAX = *(float *)para_data;
    } else if (strcmp(which_para, "T_MAX") == 0) {
        d->T_MAX = *(float *)para_data;
    }
}

/**********************************************************************************************************************/
/* M3508——速度控制 */

struct M3508_data {
    // 控制量
    int16_t _v_des;          // 目标转速 (单位: rpm)
    int16_t _current_output; // PID 计算出的电流输出 (-16384 ~ 16384)

    // PID 参数
    float _kp;
    float _ki;
    float _kd;

    // PID 内部状态
    float _i_term;           // 积分累加值
    int16_t _last_error;     // 上一次的误差，用于微分计算
    float _last_d_out;       // 上一时刻的微分输出 (用于滤波)
    float _d_filter_alpha;   // 微分项滤波系数 (取值范围 0.0~1.0)

    // 反馈量
    int16_t POS;             // 转子机械角度 (0 ~ 8191)
    int16_t VEL;             // 转子转速 (rpm)
    int16_t CURRENT;         // 实际转矩电流
    int8_t TEMP;             // 电机温度
    int8_t ERR;

    // 输出限幅
    float _current_output_max; // 最大电流输出限幅 (通常为 16384.0f)
    float _i_output_max;       // 积分项限幅 (抗饱和)

    //使能状态
    uint8_t enable_flag;
};

void M3508_VEL_PID_init(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;

    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    /* 1. 结构体清零：确保所有反馈、积分累加、微分状态从 0 开始 */
    memset(d, 0, sizeof(struct M3508_data));

    /* 2. 基础配置 */
    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);
    if (hcan != NULL) {
        motor->motor_can_handle = hcan;
    }

    /* 3. 设定要求的默认值 */
    d->_current_output_max = 500.0f;   // 默认输出限幅为 500 保证安全
    d->_i_output_max = 500.0f;         // 积分限幅通常与总输出限幅保持一致
    d->_d_filter_alpha = 1.0f;      // 默认 alpha 为 1.0 (无滤波)

    /* 4. 处理可变参数 */
    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);

        // 依次获取参数：Kp, Ki, Kd, current_max, i_max, alpha
        if (para_num >= 1) d->_kp = (float)va_arg(ap, double);
        if (para_num >= 2) d->_ki = (float)va_arg(ap, double);
        if (para_num >= 3) d->_kd = (float)va_arg(ap, double);
        if (para_num >= 4) d->_current_output_max = (float)va_arg(ap, double);
        if (para_num >= 5) d->_i_output_max = (float)va_arg(ap, double);
        if (para_num >= 6) d->_d_filter_alpha = (float)va_arg(ap, double);

        va_end(ap);
    }
    /* M3508 不需要像达妙电机那样发送特定的使能帧，
       只要持续发送电流控制帧即可开始运转 */
}

void M3508_get_measure(const struct motor_device *motor, const uint8_t *data)
{

    if (motor == NULL || motor->motor_data == NULL || data == NULL) {
        return;
    }

    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    /* 1. 解析机械角度 (0 ~ 8191) */
    d->POS = (int16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);

    /* 2. 解析转速 (RPM) */
    d->VEL = (int16_t)((uint16_t)data[2] << 8 | (uint16_t)data[3]);

    /* 3. 解析实际电流 (Torque Current) */
    d->CURRENT = (int16_t)((uint16_t)data[4] << 8 | (uint16_t)data[5]);

    /* 4. 解析温度 */
    d->TEMP = (int8_t)data[6];

    /* 5. 错误状态  */
    d->ERR = (int8_t)data[7];
}

void M3508_enable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;
    d->enable_flag = 1;
}

void M3508_disable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;
    d->enable_flag = 0;
    d->_i_term = 0.0f;       // 清空积分
    d->_current_output = 0;   // 物理输出清零
}

void M3508_VEL_PID_update(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    // 如果未使能，强制执行安全复位
    if (d->enable_flag == 0) {
        d->_i_term = 0.0f;
        d->_current_output = 0;
        return;
    }

    // 1. 计算误差
    int16_t error = d->_v_des - d->VEL;

    // 2. 比例项 (P)
    float p_out = d->_kp * (float)error;

    // 3. 积分项 (I)
    d->_i_term += d->_ki * (float)error;
    if (d->_i_term > d->_i_output_max) d->_i_term = d->_i_output_max;
    if (d->_i_term < -d->_i_output_max) d->_i_term = -d->_i_output_max;

    // 4. 微分项 (D) - 带一阶低通滤波
    float current_d_raw = d->_kd * (float)(error - d->_last_error);
    float d_out = d->_d_filter_alpha * current_d_raw + (1.0f - d->_d_filter_alpha) * d->_last_d_out;

    d->_last_error = error;
    d->_last_d_out = d_out;

    // 5. 总输出并限幅
    float total_out = p_out + d->_i_term + d_out;
    if (total_out > d->_current_output_max) total_out = d->_current_output_max;
    if (total_out < -d->_current_output_max) total_out = -d->_current_output_max;

    d->_current_output = (int16_t)total_out;
}

void M3508_VEL_PID_set_target(const struct motor_device *motor, const int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    va_list ap;
    va_start(ap, para_num);

    if (para_num >= 1) {
        // 第一个参数设为目标转速 (rpm)
        d->_v_des = (int16_t)va_arg(ap, double);
    }

    // 如果需要，可以在这里通过 para_num >= 2 修改其他运行时参数

    va_end(ap);
}

void M3508_VEL_PID_get_status(const struct motor_device *motor, const char* which_status, void* status_data) {
    if (motor == NULL || motor->motor_data == NULL || which_status == NULL || status_data == NULL) return;

    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    // 反馈量查询
    if (strcmp(which_status, "POS") == 0) {
        *(int16_t *)status_data = d->POS;
    } else if (strcmp(which_status, "VEL") == 0) {
        *(int16_t *)status_data = d->VEL;
    } else if (strcmp(which_status, "CURRENT") == 0) {
        *(int16_t *)status_data = d->CURRENT;
    } else if (strcmp(which_status, "TEMP") == 0) {
        *(int8_t *)status_data = d->TEMP;
    } else if (strcmp(which_status, "ERR") == 0) {
        *(int8_t *)status_data = d->ERR;
    }
    // 控制量与参数查询
    else if (strcmp(which_status, "v_des") == 0) {
        *(int16_t *)status_data = d->_v_des;
    } else if (strcmp(which_status, "Kp") == 0) {
        *(float *)status_data = d->_kp;
    } else if (strcmp(which_status, "Ki") == 0) {
        *(float *)status_data = d->_ki;
    } else if (strcmp(which_status, "Kd") == 0) {
        *(float *)status_data = d->_kd;
    } else if (strcmp(which_status, "current_max") == 0) {
        *(float *)status_data = d->_current_output_max;
    } else if (strcmp(which_status, "i_max") == 0) {
        *(float *)status_data = d->_i_output_max;
    } else if (strcmp(which_status, "alpha") == 0) {
        *(float *)status_data = d->_d_filter_alpha;
    }
}

void M3508_VEL_PID_set_para(const struct motor_device *motor, const char* which_para, void* para_data) {
    if (motor == NULL || motor->motor_data == NULL || which_para == NULL || para_data == NULL) return;

    struct M3508_data *d = (struct M3508_data *)motor->motor_data;

    if (strcmp(which_para, "Kp") == 0) {
        d->_kp = *(float *)para_data;
    } else if (strcmp(which_para, "Ki") == 0) {
        d->_ki = *(float *)para_data;
    } else if (strcmp(which_para, "Kd") == 0) {
        d->_kd = *(float *)para_data;
    } else if (strcmp(which_para, "current_max") == 0) {
        d->_current_output_max = *(float *)para_data;
    } else if (strcmp(which_para, "i_max") == 0) {
        d->_i_output_max = *(float *)para_data;
        // 修改限幅后，建议检查当前积分项是否超限
        if (d->_i_term > d->_i_output_max) d->_i_term = d->_i_output_max;
        if (d->_i_term < -d->_i_output_max) d->_i_term = -d->_i_output_max;
    } else if (strcmp(which_para, "alpha") == 0) {
        float a = *(float *)para_data;
        // 增加安全限制，确保 alpha 在 [0, 1] 之间
        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;
        d->_d_filter_alpha = a;
    }
}
/**********************************************************************************************************************/
/* M2006——电流控制 */
#include <string.h>
#include <stdarg.h>

/* M2006 数据结构体 */
struct M2006_data {
    // 控制量
    int16_t _v_des;          // 目标转速 (单位: rpm)
    int16_t _current_output; // PID 计算出的电流输出 (-10000 ~ 10000)

    // PID 参数
    float _kp;
    float _ki;
    float _kd;

    // PID 内部状态
    float _i_term;
    int16_t _last_error;
    float _last_d_out;
    float _d_filter_alpha;

    // 位置环（外环）
    float _kp_pos;
    float _ki_pos;
    float _i_term_pos;
    float _v_des_limit;
    int32_t _pos_deadband;

    // 多圈位置跟踪（编码器累积）
    int32_t _p_des_sum;
    int32_t _pos_sum;
    int16_t _last_pos_raw;
    uint8_t _pos_inited;

    // 反馈量
    int16_t POS;
    int16_t VEL;
    int16_t CURRENT;
    int8_t TEMP;
    int8_t ERR;

    // 输出限幅
    float _current_output_max;
    float _i_output_max;

    //使能状态
    uint8_t enable_flag;

    // 控制模式：0=位置环，1=速度环
    uint8_t ctrl_mode;
};

static void M2006_UpdatePosSum(struct M2006_data *d, int16_t pos_raw)
{
    if (d == NULL) return;

    if (d->_pos_inited == 0U) {
        d->_last_pos_raw = pos_raw;
        d->_pos_sum = pos_raw;
        d->_p_des_sum = d->_pos_sum;
        d->_pos_inited = 1U;
        return;
    }

    int32_t delta = (int32_t)pos_raw - (int32_t)d->_last_pos_raw;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;

    d->_pos_sum += delta;
    d->_last_pos_raw = pos_raw;
}

/* M2006 初始化 */
void M2006_VEL_PID_init(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;

    memset(d, 0, sizeof(struct M2006_data));

    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);
    if (hcan != NULL) motor->motor_can_handle = hcan;

    // 默认值
    d->_current_output_max = 500.0f;
    d->_i_output_max = 500.0f;
    d->_d_filter_alpha = 1.0f;
    d->_kp_pos = 1.0f;
    d->_ki_pos = 0.0f;
    d->_v_des_limit = 6000.0f;
    d->_pos_deadband = 12;
    d->ctrl_mode = 0U;

    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);
        if (para_num >= 1) d->_kp = (float)va_arg(ap, double);
        if (para_num >= 2) d->_ki = (float)va_arg(ap, double);
        if (para_num >= 3) d->_kd = (float)va_arg(ap, double);
        if (para_num >= 4) d->_current_output_max = (float)va_arg(ap, double);
        if (para_num >= 5) d->_i_output_max = (float)va_arg(ap, double);
        if (para_num >= 6) d->_d_filter_alpha = (float)va_arg(ap, double);
        va_end(ap);
    }
}

/* M2006 反馈解析 */
void M2006_get_measure(const struct motor_device *motor, const uint8_t *data)
{
    if (motor == NULL || motor->motor_data == NULL || data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;

    d->POS = (int16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
    d->VEL = (int16_t)((uint16_t)data[2] << 8 | (uint16_t)data[3]);
    d->CURRENT = (int16_t)((uint16_t)data[4] << 8 | (uint16_t)data[5]);
    d->TEMP = (int8_t)data[6];
    d->ERR = (int8_t)data[7];

    M2006_UpdatePosSum(d, d->POS);
}

void M2006_enable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    ((struct M2006_data *)motor->motor_data)->enable_flag = 1;
}

void M2006_disable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;
    d->enable_flag = 0;
    d->_i_term = 0.0f;
    d->_i_term_pos = 0.0f;
    d->_current_output = 0;
    d->_v_des = 0;
    d->ctrl_mode = 0U;
    d->_p_des_sum = d->_pos_sum;
}

void M2006_VEL_PID_update(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;

    if (d->enable_flag == 0) {
        d->_i_term = 0.0f;
        d->_current_output = 0;
        return;
    }

    if (d->ctrl_mode == 0U) {
        int32_t pos_err = d->_p_des_sum - d->_pos_sum;

        if (pos_err > d->_pos_deadband || pos_err < -d->_pos_deadband) {
            d->_i_term_pos += d->_ki_pos * (float)pos_err;
            if (d->_i_term_pos > d->_v_des_limit) d->_i_term_pos = d->_v_des_limit;
            if (d->_i_term_pos < -d->_v_des_limit) d->_i_term_pos = -d->_v_des_limit;

            float v_cmd = d->_kp_pos * (float)pos_err + d->_i_term_pos;
            if (v_cmd > d->_v_des_limit) v_cmd = d->_v_des_limit;
            if (v_cmd < -d->_v_des_limit) v_cmd = -d->_v_des_limit;
            d->_v_des = (int16_t)v_cmd;
        } else {
            d->_v_des = 0;
            d->_i_term_pos = 0.0f;
        }
    }

    int16_t error = d->_v_des - d->VEL;
    float p_out = d->_kp * (float)error;

    d->_i_term += d->_ki * (float)error;
    if (d->_i_term > d->_i_output_max) d->_i_term = d->_i_output_max;
    if (d->_i_term < -d->_i_output_max) d->_i_term = -d->_i_output_max;

    float current_d_raw = d->_kd * (float)(error - d->_last_error);
    float d_out = d->_d_filter_alpha * current_d_raw + (1.0f - d->_d_filter_alpha) * d->_last_d_out;

    d->_last_error = error;
    d->_last_d_out = d_out;

    float total_out = p_out + d->_i_term + d_out;
    if (total_out > d->_current_output_max) total_out = d->_current_output_max;
    if (total_out < -d->_current_output_max) total_out = -d->_current_output_max;

    d->_current_output = (int16_t)total_out;
}

/* M2006 目标设置 */
void M2006_VEL_PID_set_target(const struct motor_device *motor, const int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;

    va_list ap;
    va_start(ap, para_num);
    if (para_num >= 2) {
        const double v_des = va_arg(ap, double);
        const double mode = va_arg(ap, double);
        d->_v_des = (int16_t)v_des;
        d->ctrl_mode = (mode != 0.0) ? 1U : 0U;
        if (d->ctrl_mode == 0U) {
            d->_p_des_sum = d->_pos_sum;
        }
    } else if (para_num >= 1) {
        const double p_des = va_arg(ap, double);
        d->_p_des_sum = (int32_t)p_des;
        d->ctrl_mode = 0U;
    }
    va_end(ap);
}

/* M2006 状态获取 */
void M2006_VEL_PID_get_status(const struct motor_device *motor, const char* which_status, void* status_data) {
    if (motor == NULL || motor->motor_data == NULL || which_status == NULL || status_data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;

    if (strcmp(which_status, "POS") == 0) *(int16_t *)status_data = d->POS;
    else if (strcmp(which_status, "VEL") == 0) *(int16_t *)status_data = d->VEL;
    else if (strcmp(which_status, "CURRENT") == 0) *(int16_t *)status_data = d->CURRENT;
    else if (strcmp(which_status, "POS_SUM") == 0) *(int32_t *)status_data = d->_pos_sum;
    else if (strcmp(which_status, "TEMP") == 0) *(int8_t *)status_data = d->TEMP;
    else if (strcmp(which_status, "ERR") == 0) *(int8_t *)status_data = d->ERR;
    else if (strcmp(which_status, "v_des") == 0) *(int16_t *)status_data = d->_v_des;
    else if (strcmp(which_status, "Kp") == 0) *(float *)status_data = d->_kp;
    else if (strcmp(which_status, "Ki") == 0) *(float *)status_data = d->_ki;
    else if (strcmp(which_status, "Kd") == 0) *(float *)status_data = d->_kd;
    else if (strcmp(which_status, "current_max") == 0) *(float *)status_data = d->_current_output_max;
    else if (strcmp(which_status, "i_max") == 0) *(float *)status_data = d->_i_output_max;
    else if (strcmp(which_status, "alpha") == 0) *(float *)status_data = d->_d_filter_alpha;
}

void M2006_VEL_PID_set_para(const struct motor_device *motor, const char* which_para, void* para_data) {
    if (motor == NULL || motor->motor_data == NULL || which_para == NULL || para_data == NULL) return;
    struct M2006_data *d = (struct M2006_data *)motor->motor_data;

    if (strcmp(which_para, "Kp") == 0) d->_kp = *(float *)para_data;
    else if (strcmp(which_para, "Ki") == 0) d->_ki = *(float *)para_data;
    else if (strcmp(which_para, "Kd") == 0) d->_kd = *(float *)para_data;
    else if (strcmp(which_para, "Kp_pos") == 0) d->_kp_pos = *(float *)para_data;
    else if (strcmp(which_para, "Ki_pos") == 0) d->_ki_pos = *(float *)para_data;
    else if (strcmp(which_para, "v_des_limit") == 0) d->_v_des_limit = *(float *)para_data;
    else if (strcmp(which_para, "pos_deadband") == 0) d->_pos_deadband = *(int32_t *)para_data;
    else if (strcmp(which_para, "current_max") == 0) d->_current_output_max = *(float *)para_data;
    else if (strcmp(which_para, "i_max") == 0) {
        d->_i_output_max = *(float *)para_data;
        if (d->_i_term > d->_i_output_max) d->_i_term = d->_i_output_max;
        if (d->_i_term < -d->_i_output_max) d->_i_term = -d->_i_output_max;
    }
    else if (strcmp(which_para, "alpha") == 0) {
        float a = *(float *)para_data;
        if (a < 0.0f) a = 0.0f; if (a > 1.0f) a = 1.0f;
        d->_d_filter_alpha = a;
    }
}

/**********************************************************************************************************************/
/* GM6020——速度位置控制 */

#define GM6020_ANGLE_TO_RAD (2.0f * M_PI / 8192.0f)
#define GM6020_RAD_TO_ANGLE (8192.0f / (2.0f * M_PI))

struct GM6020_data {
    // 目标量
    float   _p_des;          // 目标位置 (rad)
    float   _v_des_internal; // 内部速度目标 (位置环输出)
    float   _v_des;          // 目标速度 (rpm)
    int16_t _out_output;     // 最终电压输出 (-30000 ~ 30000)

    // 位置环 PID (外环)
    float _kp_p;
    float _ki_p;
    float _kd_p;             // 位置环阻尼项（基于速度反馈）
    float _i_term_p;
    float _i_p_max;

    // 速度环 PID (内环)
    float _kp_v;
    float _ki_v;
    float _kd_v;
    float _i_term_v;
    float _i_v_max;
    float _last_d_out;
    float _d_filter_alpha;
    int16_t _last_v_error;

    // 独立速度前馈
    float _kp_v_only;

    // 反馈量
    int16_t POS;             // 机械角度 (0 ~ 8191)
    int16_t VEL;             // 转速 (rpm)
    int16_t TORQUE;          // 转矩反馈
    int8_t  TEMP;

    // 限制与安全
    float   _v_limit;        // 最大限制转速 (rpm)
    float   _out_max;        // 最大输出电压

    // 使能状态
    uint8_t enable_flag;
};

/* 初始化 */
void GM6020_PV_init(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...)
{
    if (motor == NULL || motor->motor_data == NULL) return;
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;

    /* 1. 全清零，防止脏数据导致电机疯转 */
    memset(d, 0, sizeof(struct GM6020_data));

    /* 2. 基础配置 */
    motor->motor_id = (uint32_t)(motor_ID & 0x7FFU);
    if (hcan != NULL) motor->motor_can_handle = hcan;

    /* 3. 设定默认安全值 (如果外部没传参) */
    d->_v_limit = 320.0f;        // GM6020 额定转速
    d->_out_max = 5000.0f;       // 初始安全电压限幅
    d->_i_p_max = 100.0f;        // 位置环积分限幅
    d->_i_v_max = 5000.0f;       // 速度环积分限幅
    d->_d_filter_alpha = 1.0f;   // 默认关闭滤波

    /* 4. 补全参数映射 (按照 8 个参数设计) */
    if (para_num > 0) {
        va_list ap;
        va_start(ap, para_num);

        // 顺序：P_Kp, P_Ki, V_Kp, V_Ki, V_Kd, Out_Max, V_Limit, Alpha, V_Only_Kp, [可选]P_Kd
        if (para_num >= 1) d->_kp_p = (float)va_arg(ap, double);
        if (para_num >= 2) d->_ki_p = (float)va_arg(ap, double);

        if (para_num >= 3) d->_kp_v = (float)va_arg(ap, double);
        if (para_num >= 4) d->_ki_v = (float)va_arg(ap, double);
        if (para_num >= 5) d->_kd_v = (float)va_arg(ap, double);

        if (para_num >= 6) d->_out_max = (float)va_arg(ap, double);
        if (para_num >= 7) d->_v_limit = (float)va_arg(ap, double);
        if (para_num >= 8) d->_d_filter_alpha = (float)va_arg(ap, double);
        if (para_num >= 9) d->_kp_v_only = (float)va_arg(ap, double);
        if (para_num >= 10) d->_kd_p = (float)va_arg(ap, double);

        va_end(ap);
    }
}

/* 反馈解析 */
void GM6020_get_measure(const struct motor_device *motor, const uint8_t *data) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;
    d->POS = (int16_t)((uint16_t)data[0] << 8 | (uint16_t)data[1]);
    d->VEL = (int16_t)((uint16_t)data[2] << 8 | (uint16_t)data[3]);
    d->TORQUE = (int16_t)((uint16_t)data[4] << 8 | (uint16_t)data[5]);
    d->TEMP = (int8_t)data[6];
}

void GM6020_enable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    ((struct GM6020_data *)motor->motor_data)->enable_flag = 1;
}

void GM6020_disable(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;
    d->enable_flag = 0;
    d->_i_term_p = 0.0f;
    d->_i_term_v = 0.0f;
    d->_out_output = 0;

    // [重要] 失能时将目标位置重置为当前反馈位置，防止下次使能时猛甩
    d->_p_des = (float)d->POS * GM6020_ANGLE_TO_RAD;
}

// 纯速度位置环
void GM6020_PV_update(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;

    if (d->enable_flag == 0) {
        d->_i_term_p = 0.0f;
        d->_i_term_v = 0.0f;
        d->_out_output = 0;
        // 持续同步目标值
        d->_p_des = (float)d->POS * GM6020_ANGLE_TO_RAD;
        return;
    }

    // 1. 位置环计算
    float current_p_rad = (float)d->POS * GM6020_ANGLE_TO_RAD;
    float p_error = d->_p_des - current_p_rad;

    // 最短路径处理
    while (p_error > M_PI)  p_error -= 2.0f * M_PI;
    while (p_error < -M_PI) p_error += 2.0f * M_PI;

    d->_i_term_p += d->_ki_p * p_error;
    if (d->_i_term_p > d->_i_p_max) d->_i_term_p = d->_i_p_max;
    if (d->_i_term_p < -d->_i_p_max) d->_i_term_p = -d->_i_p_max;

    d->_v_des_internal = d->_kp_p * p_error + d->_i_term_p - d->_kd_p * (float)d->VEL;

    // 速度限制
    if (d->_v_des_internal > d->_v_limit) d->_v_des_internal = d->_v_limit;
    if (d->_v_des_internal < -d->_v_limit) d->_v_des_internal = -d->_v_limit;

    // 2. 速度环计算
    float v_error = d->_v_des_internal - (float)d->VEL;
    float v_p_out = d->_kp_v * v_error;

    d->_i_term_v += d->_ki_v * v_error;
    if (d->_i_term_v > d->_i_v_max) d->_i_term_v = d->_i_v_max;
    if (d->_i_term_v < -d->_i_v_max) d->_i_term_v = -d->_i_v_max;

    float d_raw = d->_kd_v * (v_error - (float)d->_last_v_error);
    float v_d_out = d->_d_filter_alpha * d_raw + (1.0f - d->_d_filter_alpha) * d->_last_d_out;

    d->_last_v_error = (int16_t)v_error;
    d->_last_d_out = v_d_out;

    // 3. 输出限幅
    float total_out = v_p_out + d->_i_term_v + v_d_out;
    if (total_out > d->_out_max) total_out = d->_out_max;
    if (total_out < -d->_out_max) total_out = -d->_out_max;

    d->_out_output = (int16_t)total_out;
}

// 速度位置环 并 速度环
void GM6020_PV_V_update(struct motor_device *motor) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;

    if (d->enable_flag == 0) {
        d->_i_term_p = 0.0f;
        d->_i_term_v = 0.0f;
        d->_out_output = 0;
        // 持续同步目标值
        d->_p_des = (float)d->POS * GM6020_ANGLE_TO_RAD;
        return;
    }

    // 1. 位置环计算
    float current_p_rad = (float)d->POS * GM6020_ANGLE_TO_RAD;
    float p_error = d->_p_des - current_p_rad;

    // 最短路径处理
    while (p_error > M_PI)  p_error -= 2.0f * M_PI;
    while (p_error < -M_PI) p_error += 2.0f * M_PI;

    d->_i_term_p += d->_ki_p * p_error;
    if (d->_i_term_p > d->_i_p_max) d->_i_term_p = d->_i_p_max;
    if (d->_i_term_p < -d->_i_p_max) d->_i_term_p = -d->_i_p_max;

    d->_v_des_internal = d->_kp_p * p_error + d->_i_term_p - d->_kd_p * (float)d->VEL;

    // 速度限制
    if (d->_v_des_internal > d->_v_limit) d->_v_des_internal = d->_v_limit;
    if (d->_v_des_internal < -d->_v_limit) d->_v_des_internal = -d->_v_limit;

    // 2. 速度环计算
    float v_error = d->_v_des_internal - (float)d->VEL;
    float v_p_out = d->_kp_v * v_error;

    d->_i_term_v += d->_ki_v * v_error;
    if (d->_i_term_v > d->_i_v_max) d->_i_term_v = d->_i_v_max;
    if (d->_i_term_v < -d->_i_v_max) d->_i_term_v = -d->_i_v_max;

    float d_raw = d->_kd_v * (v_error - (float)d->_last_v_error);
    float v_d_out = d->_d_filter_alpha * d_raw + (1.0f - d->_d_filter_alpha) * d->_last_d_out;

    d->_last_v_error = (int16_t)v_error;
    d->_last_d_out = v_d_out;

    // 3. 纯速度环计算
    float v_only_out = d->_v_des * d->_kp_v_only;

    // 3. 输出限幅
    float total_out = v_p_out + d->_i_term_v + v_d_out + v_only_out;
    if (total_out > d->_out_max) total_out = d->_out_max;
    if (total_out < -d->_out_max) total_out = -d->_out_max;

    d->_out_output = (int16_t)total_out;
}

/* 设定目标 (rad) */
void GM6020_PV_set_target(const struct motor_device *motor, const int para_num, ...) {
    if (motor == NULL || motor->motor_data == NULL) return;
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;
    va_list ap; va_start(ap, para_num);
    if (para_num >= 1) d->_p_des = (float)va_arg(ap, double);
    if (para_num >= 2) d->_v_des = (float)va_arg(ap, double);
    va_end(ap);
}

/* 获取状态 */
void GM6020_PV_get_status(const struct motor_device *motor, const char* which_status, void* status_data) {
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;
    if (strcmp(which_status, "POS") == 0)      *(float *)status_data = (float)d->POS * GM6020_ANGLE_TO_RAD;
    else if (strcmp(which_status, "VEL") == 0) *(int16_t *)status_data = d->VEL;
    else if (strcmp(which_status, "p_des") == 0) *(float *)status_data = d->_p_des;
    else if (strcmp(which_status, "Kp_p") == 0) *(float *)status_data = d->_kp_p;
    else if (strcmp(which_status, "Kd_p") == 0) *(float *)status_data = d->_kd_p;
    else if (strcmp(which_status, "Kp_v") == 0) *(float *)status_data = d->_kp_v;
    else if (strcmp(which_status, "out_max") == 0) *(float *)status_data = d->_out_max;
}

/* 设置参数 */
void GM6020_PV_set_para(const struct motor_device *motor, const char* which_para, void* para_data) {
    struct GM6020_data *d = (struct GM6020_data *)motor->motor_data;
    if (strcmp(which_para, "Kp_p") == 0)      d->_kp_p = *(float *)para_data;
    else if (strcmp(which_para, "Ki_p") == 0) d->_ki_p = *(float *)para_data;
    else if (strcmp(which_para, "Kd_p") == 0) d->_kd_p = *(float *)para_data;
    else if (strcmp(which_para, "Kp_v") == 0) d->_kp_v = *(float *)para_data;
    else if (strcmp(which_para, "Ki_v") == 0) d->_ki_v = *(float *)para_data;
    else if (strcmp(which_para, "Kd_v") == 0) d->_kd_v = *(float *)para_data;
    else if (strcmp(which_para, "Kp_v_only") == 0) d->_kp_v_only = *(float *)para_data;
    else if (strcmp(which_para, "out_max") == 0) d->_out_max = *(float *)para_data;
    else if (strcmp(which_para, "v_limit") == 0) d->_v_limit = *(float *)para_data;
}

/**********************************************************************************************************************/
/*实例*/

#include "motor.h"

/* --- 1. 达妙电机 (Pitch) --- */
struct DM_MIT_data J4310_PITCH_data = {0};
struct motor_device J4310_PITCH = {
    .motor_name = "J4310_PITCH",
    .motor_id = 0x01,
    .motor_can_handle = NULL, // 假设达妙在CAN2
    .motor_data = &J4310_PITCH_data,
    .init = DM_MIT_init,
    .get_measure = DM_get_measure,
    .update = DM_update,
    .send_ctrl_cmd = DM_MIT_send_ctrl_cmd,
    .send_disable_cmd = DM_MIT_send_disable_cmd,
    .send_enable_cmd = DM_MIT_send_enable_cmd,
    .set_target = DM_MIT_set_target,
    .get_status = DM_MIT_get_status,
    .set_para = DM_MIT_set_para
};

/* --- 2. 底盘电机 (M3508 x 4) --- */
struct M3508_data M3508_CHASSIS_1_data = {0};
struct motor_device M3508_CHASSIS_1 = {
    .motor_name = "M3508_CHASSIS_1",
    .motor_id = 0x201,
    .motor_can_handle = NULL,
    .motor_data = &M3508_CHASSIS_1_data,
    .init = M3508_VEL_PID_init,
    .get_measure = M3508_get_measure,
    .update = M3508_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,
    .send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,
    .get_status = M3508_VEL_PID_get_status,
    .set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_CHASSIS_2_data = {0};
struct motor_device M3508_CHASSIS_2 = {
    .motor_name = "M3508_CHASSIS_2",
    .motor_id = 0x202,
    .motor_can_handle = NULL,
    .motor_data = &M3508_CHASSIS_2_data,
    .init = M3508_VEL_PID_init,
    .get_measure = M3508_get_measure,
    .update = M3508_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,
    .send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,
    .get_status = M3508_VEL_PID_get_status,
    .set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_CHASSIS_3_data = {0};
struct motor_device M3508_CHASSIS_3 = {
    .motor_name = "M3508_CHASSIS_3",
    .motor_id = 0x203,
    .motor_can_handle = NULL,
    .motor_data = &M3508_CHASSIS_3_data,
    .init = M3508_VEL_PID_init,
    .get_measure = M3508_get_measure,
    .update = M3508_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,
    .send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,
    .get_status = M3508_VEL_PID_get_status,
    .set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_CHASSIS_4_data = {0};
struct motor_device M3508_CHASSIS_4 = {
    .motor_name = "M3508_CHASSIS_4",
    .motor_id = 0x204,
    .motor_can_handle = NULL,
    .motor_data = &M3508_CHASSIS_4_data,
    .init = M3508_VEL_PID_init,
    .get_measure = M3508_get_measure,
    .update = M3508_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,
    .send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,
    .get_status = M3508_VEL_PID_get_status,
    .set_para = M3508_VEL_PID_set_para
};

/* --- 3. 拨弹电机 (M2006) --- */
struct M2006_data M2006_TRIGGER_data = {0};
struct motor_device M2006_TRIGGER = {
    .motor_name = "M2006_TRIGGER",
    .motor_id = 0x205,
    .motor_can_handle = NULL,
    .motor_data = &M2006_TRIGGER_data,
    .init = M2006_VEL_PID_init,
    .get_measure = M2006_get_measure,
    .update = M2006_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M2006_disable,
    .send_enable_cmd = M2006_enable,
    .set_target = M2006_VEL_PID_set_target,
    .get_status = M2006_VEL_PID_get_status,
    .set_para = M2006_VEL_PID_set_para
};

/* --- 4. 云台 Yaw (GM6020) --- */
struct GM6020_data GM6020_YAW_data = {0};
struct motor_device GM6020_YAW = {
    .motor_name = "GM6020_YAW",
    .motor_id = 0x206,
    .motor_can_handle = NULL,
    .motor_data = &GM6020_YAW_data,
    .init = GM6020_PV_init,
    .get_measure = GM6020_get_measure,
    .update = GM6020_PV_V_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = GM6020_disable,
    .send_enable_cmd = GM6020_enable,
    .set_target = GM6020_PV_set_target,
    .get_status = GM6020_PV_get_status,
    .set_para = GM6020_PV_set_para
};

/* --- 5. 摩擦轮电机 (M3508 x 2) --- */
struct M3508_data M3508_SHOOT_R_data = {0};
struct motor_device M3508_SHOOT_R = {
    .motor_name = "M3508_SHOOT_R",
    .motor_id = 0x201,
    .motor_can_handle = NULL,
    .motor_data = &M3508_SHOOT_R_data,
    .init = M3508_VEL_PID_init,
    .get_measure = M3508_get_measure,
    .update = M3508_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,
    .send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,
    .get_status = M3508_VEL_PID_get_status,
    .set_para = M3508_VEL_PID_set_para
};

struct M3508_data M3508_SHOOT_L_data = {0};
struct motor_device M3508_SHOOT_L = {
    .motor_name = "M3508_SHOOT_L",
    .motor_id = 0x202,
    .motor_can_handle = NULL,
    .motor_data = &M3508_SHOOT_L_data,
    .init = M3508_VEL_PID_init,
    .get_measure = M3508_get_measure,
    .update = M3508_VEL_PID_update,
    .send_ctrl_cmd = NULL,
    .send_disable_cmd = M3508_disable,
    .send_enable_cmd = M3508_enable,
    .set_target = M3508_VEL_PID_set_target,
    .get_status = M3508_VEL_PID_get_status,
    .set_para = M3508_VEL_PID_set_para
};
/**********************************************************************************************************************/
/*对外接口*/
struct motor_device *motor_list[] = {
    &J4310_PITCH,
    &M3508_CHASSIS_1,
    &M3508_CHASSIS_2,
    &M3508_CHASSIS_3,
    &M3508_CHASSIS_4,
    &M2006_TRIGGER,
    &GM6020_YAW,
    &M3508_SHOOT_L,
    &M3508_SHOOT_R,
};
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/* 电机实例与列表管理 (Static) */

#ifndef MOTOR_COUNT
#define MOTOR_COUNT (sizeof(motor_list) / sizeof(motor_list[0]))
#endif

/**
 * @brief 根据名称获取电机设备指针 (对外接口)
 */
struct motor_device *motor_get_device(const char *name)
{
    if (name == NULL) return NULL;
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
        if (motor_list[i] != NULL && motor_list[i]->motor_name != NULL) {
            if (strcmp(motor_list[i]->motor_name, name) == 0) {
                return motor_list[i];
            }
        }
    }
    return NULL;
}

uint32_t Motor_Get_Count(void)
{
    return (uint32_t)MOTOR_COUNT;
}

/**********************************************************************************************************************/
/* 私有内部逻辑 */

/**
 * @brief 配置所有电机的初始 PID 参数 (私有)
 */
static void All_Motors_Init(void) {
    // 1. 达妙 PITCH (MIT模式) - 注意使用 CAN2
    // 参数含义: [ID, 句柄, 参数个数, Kp, Kd, P_MAX, V_MAX, T_MAX]
    J4310_PITCH.init(&J4310_PITCH, 0x01, &hcan2, 5,
                     18.0,  /* Kp: 降低位置刚性，减小“掰着走”阻力 */
                     0.8,   /* Kd: 显著减小速度阻尼，改善手感 */
                     12.5,  /* P_MAX: 位置限幅 (rad) */
                     4.0,   /* V_MAX: 略放宽速度限幅，减少跟随拖滞 */
                     8.0    /* T_MAX: 降低最大扭矩，避免动作发硬 */
    );

    // 2. 底盘 M3508 (1-4): 速度环 PID
    // 参数含义: [ID, 句柄, 参数个数, Kp, Ki, Kd, Max_Out, I_Max, Alpha]
    const char* chassis_names[] = {"M3508_CHASSIS_1", "M3508_CHASSIS_2", "M3508_CHASSIS_3", "M3508_CHASSIS_4"};
    for (int i = 0; i < 4; i++) {
        struct motor_device *m = motor_get_device(chassis_names[i]);
        if (m) {
            m->init(m, 0x201 + i, &hcan1, 6,
                    10.0,     /* Kp */
                    0.0,      /* Ki */
                    0.0,      /* Kd */
                    10000.0,  /* Max_Out: 最大电流输出 (max 16384) */
                    2000.0,   /* I_Max: 积分限幅 */
                    0.5       /* Alpha: 微分项低通滤波系数 */
            );
        }
    }

    // 3. M2006 拨弹电机 (速度环)
    M2006_TRIGGER.init(&M2006_TRIGGER, 0x205, &hcan1, 6,
                       10.0,     /* Kp */
                       0.1,      /* Ki */
                       0.0,      /* Kd */
                       10000.0,  /* Max_Out */
                       3000.0,   /* I_Max */
                       1.0       /* Alpha */
    );

    // 4. GM6020 YAW轴 (位置-速度串级)
    // 参数含义: [ID, 句柄, 参数个数, P_Kp, P_Ki, V_Kp, V_Ki, V_Kd, Out_Max, V_Limit, Alpha, V_Only_Kp, P_Kd]
    GM6020_YAW.init(&GM6020_YAW, 0x206, &hcan1, 10,
                    420.0,     /* P_Kp */
                    0.0,       /* P_Ki */
                    200.0,     /* V_Kp */
                    0.0,       /* V_Ki */
                    0.0,       /* V_Kd */
                    25000.0,   /* Out_Max */
                    320.0,     /* V_Limit */
                    1.0,       /* Alpha */
                    300.0,     /* V Only Kp */
                    1.45       /* P_Kd: 一级位置阻尼 */
    );

    // 5. 摩擦轮电机 M3508 (速度环)
    M3508_SHOOT_L.init(&M3508_SHOOT_L, 0x201, &hcan2, 6, 20.0, 0.0, 0.0, 16384.0, 5000.0, 0.3);
    M3508_SHOOT_R.init(&M3508_SHOOT_R, 0x202, &hcan2, 6, 20.0, 0.0, 0.0, 16384.0, 5000.0, 0.3);
}

/**
 * @brief 同步 des = POS，确保上电不跳动 (私有)
 */
static void Motor_Internal_Sync(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        struct motor_device *m = motor_list[i];
        if (!m || !m->motor_data) continue;

        // 统一逻辑：同步时先失能，清空积分，防止上电抖动
        if (m->send_disable_cmd) m->send_disable_cmd(m);

        if (strstr(m->motor_name, "GM6020")) {
            struct GM6020_data *d = (struct GM6020_data *)m->motor_data;
            d->_p_des = (float)d->POS * GM6020_ANGLE_TO_RAD;
        } else if (strstr(m->motor_name, "J4310")) {
            struct DM_MIT_data *d = (struct DM_MIT_data *)m->motor_data;
            d->_p_des = 0; d->_v_des = 0; d->_t_ff = 0;
        } else if (strstr(m->motor_name, "M2006")) {
            struct M2006_data *d = (struct M2006_data *)m->motor_data;
            d->_v_des = 0;
            d->_p_des_sum = d->_pos_sum;
            d->ctrl_mode = 0U;
        } else {
            // M3508
            struct M3508_data *d = (struct M3508_data *)m->motor_data;
            d->_v_des = 0;
        }
    }
}

/**********************************************************************************************************************/
/* 对外控制接口 */

/**
 * @brief 系统上电初始化全家桶 (在 RTOS 启动任务调用)
 */
void Motor_System_PowerOn_Init(void) {
    All_Motors_Init();
    osDelay(50);             // 等待 CAN 总线反馈填充
    Motor_Internal_Sync();

    // 使能达妙 (CAN2)
    struct motor_device *pitch = motor_get_device("J4310_PITCH");
    if (pitch && pitch->send_enable_cmd) pitch->send_enable_cmd(pitch);
}

/**
 * @brief 周期性执行所有电机的 PID 计算 (1ms)
 */
void Motor_All_Update(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motor_list[i] && motor_list[i]->update) {
            motor_list[i]->update(motor_list[i]);
        }
    }
}

/**
 * @brief 大疆电机组帧发送 (专供 CAN1 总线)
 * @note  处理 StdId 0x200 (底盘1-4) 和 0x1FF (云台YAW、拨弹)
 * @param hcan CAN硬件句柄指针，内部会校验确保是 hcan1
 */
void DJI_Motor_Send_CAN1_Group(CAN_HandleTypeDef *hcan) {
    /* 1. 安全校验：防止句柄传错导致硬件总线冲突 */
    if (hcan != &hcan1) return;

    uint8_t tx_200[8] = {0}, tx_1FF[8] = {0};
    CAN_TxHeaderTypeDef header = {
        .IDE = CAN_ID_STD,   // 标准帧
        .RTR = CAN_RTR_DATA, // 数据帧
        .DLC = 8             // 固定长度8字节
    };
    uint32_t mailbox;

    /* 2. 组 0x200 帧：底盘 M3508 电机 (ID: 1, 2, 3, 4) */
    for (int i = 0; i < 4; i++) {
        char name[32];
        sprintf(name, "M3508_CHASSIS_%d", i + 1);
        struct motor_device *m = motor_get_device(name);
        if (m && m->motor_data) {
            struct M3508_data *d = (struct M3508_data *)m->motor_data;
            int16_t out = (d->enable_flag) ? d->_current_output : 0;
            // 大疆协议：高位在前 (Big-Endian)
            tx_200[i*2]   = (uint8_t)(out >> 8);
            tx_200[i*2+1] = (uint8_t)(out & 0xFF);
        }
    }
    header.StdId = 0x200;
    HAL_CAN_AddTxMessage(hcan, &header, tx_200, &mailbox);

    /* 3. 组 0x1FF 帧：其他执行机构 (ID: 5, 6) */
    // 索引映射说明：i=0->ID 5(拨弹), i=1->ID 6(云台YAW)
    const char* group1_1FF[] = {"M2006_TRIGGER", "GM6020_YAW", NULL, NULL};
    for (int i = 0; i < 4; i++) {
        if (group1_1FF[i] == NULL) continue;

        struct motor_device *m = motor_get_device(group1_1FF[i]);
        if (m && m->motor_data) {
            int16_t out = 0;
            // 兼容性处理：GM6020 发送的是电压值，M3508/2006 发送的是电流值
            if (strstr(m->motor_name, "GM6020")) {
                out = ((struct GM6020_data *)m->motor_data)->_out_output;
            } else {
                out = ((struct M3508_data *)m->motor_data)->_current_output;
            }
            tx_1FF[i*2]   = (uint8_t)(out >> 8);
            tx_1FF[i*2+1] = (uint8_t)(out & 0xFF);
        }
    }
    header.StdId = 0x1FF;
    HAL_CAN_AddTxMessage(hcan, &header, tx_1FF, &mailbox);
}

/**
 * @brief 大疆电机组帧发送 (专供 CAN2 总线)
 * @note  主要负责发射机构 (摩擦轮 Shoot_L, Shoot_R) 的控制
 */
void DJI_Motor_Send_CAN2_Group(CAN_HandleTypeDef *hcan) {
    /* 1. 安全校验 */
    if (hcan != &hcan2) return;

    uint8_t tx_200[8] = {0};
    CAN_TxHeaderTypeDef header = {
        .StdId = 0x200,      // 摩擦轮 ID 通常拨码为 1 和 2，对应 0x200 控制组
        .IDE   = CAN_ID_STD,
        .RTR   = CAN_RTR_DATA,
        .DLC   = 8
    };
    uint32_t mailbox;

    /* 2. 获取并填充发射机构数据 */

    // --- 摩擦轮左 (ID 1) ---
    struct motor_device *m_l = motor_get_device("M3508_SHOOT_L");
    if (m_l && m_l->motor_data) {
        struct M3508_data *d = (struct M3508_data *)m_l->motor_data;
        // [新增] 使能判断逻辑
        int16_t out = (d->enable_flag) ? d->_current_output : 0;
        tx_200[0] = (uint8_t)(out >> 8);
        tx_200[1] = (uint8_t)(out & 0xFF);
    }

    // --- 摩擦轮右 (ID 2) ---
    struct motor_device *m_r = motor_get_device("M3508_SHOOT_R");
    if (m_r && m_r->motor_data) {
        struct M3508_data *d = (struct M3508_data *)m_r->motor_data;
        // [新增] 使能判断逻辑
        int16_t out = (d->enable_flag) ? d->_current_output : 0;
        tx_200[2] = (uint8_t)(out >> 8);
        tx_200[3] = (uint8_t)(out & 0xFF);
    }

    /* 3. 执行发送 */
    // 即使电机离线或失能，也发送全 0 帧，确保电调接收到显式的停止指令
    HAL_CAN_AddTxMessage(hcan, &header, tx_200, &mailbox);
}

/**
 * @brief CAN 中断接收回调 (支持 CAN1/CAN2 双总线)
 * @note  CAN1: 底盘, 云台, 拨弹
 * @note  CAN2: 达妙PITCH, 发射机构(Shoot_L/R)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 获取 CAN 消息
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) return;

    /* --- 解析来自对端单片机的比赛/电容信息（CAN1: 0x301/0x302） --- */
    if (hcan == &hcan1) {
        if (rx_header.StdId == 0x301U) {
            robot_ctrl.game_info.robot_id = rx_data[0];
            robot_ctrl.game_info.game_progress = rx_data[1];
            robot_ctrl.game_info.stage_remain_time = (uint16_t)rx_data[2] | ((uint16_t)rx_data[3] << 8);
            robot_ctrl.game_info.current_HP = (uint16_t)rx_data[4] | ((uint16_t)rx_data[5] << 8);
            robot_ctrl.game_info.capacity_voltage = (int16_t)((uint16_t)rx_data[6] | ((uint16_t)rx_data[7] << 8));
            robot_ctrl.game_info.last_tick_301 = osKernelSysTick();
            robot_ctrl.game_info.online_301 = 1U;
            return;
        }

        if (rx_header.StdId == 0x302U) {
            robot_ctrl.game_info.shooter_17mm_barrel_heat = (uint16_t)rx_data[0] | ((uint16_t)rx_data[1] << 8);
            robot_ctrl.game_info.armor_id = (uint8_t)(rx_data[2] & 0x0FU);
            robot_ctrl.game_info.center_bonus_state = (uint8_t)(rx_data[3] & 0x03U);
            robot_ctrl.game_info.rfid_supply19 = (uint8_t)(rx_data[4] & 0x01U);
            robot_ctrl.game_info.rfid_center23 = (uint8_t)(rx_data[5] & 0x01U);
            robot_ctrl.game_info.last_tick_302 = osKernelSysTick();
            robot_ctrl.game_info.online_302 = 1U;
            return;
        }
    }

    /* --- 处理 CAN2 总线 (达妙 + 发射机构) --- */
    if (hcan == &hcan2) {
        // 1. 达妙电机反馈 (达妙反馈 ID 通常为 0x00，内部通过 Data[0] 区分 ID)
        if (rx_header.StdId == 0x00) {
            struct motor_device *dm = motor_get_device("J4310_PITCH");
            // 校验反馈帧中的 ID 是否匹配实例 ID
            if (dm && (rx_data[0] & 0x0F) == dm->motor_id) {
                dm->get_measure(dm, rx_data);
                dm->last_rx_tick = osKernelSysTick();
            }
        }
        // 2. 大疆电机反馈 (Shoot_L/R 挂在 CAN2, ID 为 0x201/0x202)
        else if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208) {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (motor_list[i] &&
                    motor_list[i]->motor_id == rx_header.StdId &&
                    motor_list[i]->motor_can_handle == &hcan2) {
                    motor_list[i]->get_measure(motor_list[i], rx_data);
                    motor_list[i]->last_rx_tick = osKernelSysTick();
                    break;
                    }
            }
        }
        return;
    }

    /* --- 处理 CAN1 总线 (大疆底盘、云台、拨弹) --- */
    if (hcan == &hcan1) {
        // 大疆 ID 范围解析
        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208) {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (motor_list[i] &&
                    motor_list[i]->motor_id == rx_header.StdId &&
                    motor_list[i]->motor_can_handle == &hcan1) {
                    motor_list[i]->get_measure(motor_list[i], rx_data);
                    motor_list[i]->last_rx_tick = osKernelSysTick();
                    break;
                    }
            }
        }
    }
}
