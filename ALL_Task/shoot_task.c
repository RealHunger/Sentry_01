#include "shoot_task.h"

#include "cmsis_os.h"
#include "math.h"

#include "../Application/robot_global.h"
#include "../Components/motor/motor.h"
#include "../Components/remote/remote.h"

/* 500Hz shoot control parameters */
#define SHOOT_FW_SPEED         6000.0f
#define STIR_REVERSE_SPEED     2500.0f
#define SHOOT_HEAT_LIMIT_17MM  30U
#define SHOOT_TASK_PERIOD_MS   2U

/* 长按连发参数：先单发，再按住一段时间进入连发 */
#define FIRE_HOLD_START_MS     180U
#define FIRE_BURST_INTERVAL_MS 30U
#define FIRE_HOLD_START_TICKS  (FIRE_HOLD_START_MS / SHOOT_TASK_PERIOD_MS)
#define FIRE_BURST_TICKS       (FIRE_BURST_INTERVAL_MS / SHOOT_TASK_PERIOD_MS)

/* 自瞄shoot上升沿触发三连发参数 */
#define AUTO_BURST_SHOTS        3U
#define AUTO_BURST_INTERVAL_MS  200U
#define AUTO_BURST_TICKS        (AUTO_BURST_INTERVAL_MS / SHOOT_TASK_PERIOD_MS)
#define STIR_TARGET_HOLD_TOL    800

/* Trigger plate step config (output side). */
#define STIR_ENCODER_CPR           8192.0f
#define STIR_GEAR_RATIO            36.0f   /* 减速箱 36:1 */
#define STIR_BELT_RATIO            2.8f    /* 同步带比，可在线调整 */
#define STIR_TOTAL_RATIO           (STIR_GEAR_RATIO * STIR_BELT_RATIO)
#define STIR_STEP_OUTPUT_DEG       36.0f   /* 单击拨弹角度（输出侧�� */
#define STIR_STEP_DIR              (-1.0f) /* 方向不对改为 +1.0f */

#define STIR_STEP_TICKS ((int32_t)(STIR_STEP_DIR * STIR_ENCODER_CPR * STIR_TOTAL_RATIO * (STIR_STEP_OUTPUT_DEG / 360.0f)))

/* 卡弹堵转自救参数（按实机可继续微调） */
#define STIR_JAM_CURRENT_THRESH   3500
#define STIR_JAM_VEL_THRESH       120
#define STIR_JAM_DETECT_TICKS     30U   /* 60ms @500Hz */
#define STIR_JAM_REVERSE_TICKS    80U   /* 160ms @500Hz */
#define STIR_JAM_COOLDOWN_TICKS   40U   /* 80ms @500Hz */

typedef enum {
	STIR_JAM_IDLE = 0,
	STIR_JAM_RECOVER,
	STIR_JAM_COOLDOWN,
} stir_jam_state_e;

void shoot_task_func(void const * argument)
{
	struct motor_device *shoot_l = motor_get_device("M3508_SHOOT_L");
	struct motor_device *shoot_r = motor_get_device("M3508_SHOOT_R");
	struct motor_device *stir_m  = motor_get_device("M2006_TRIGGER");

	int32_t stir_pos_sum = 0;
	int32_t stir_target_sum = 0;
	uint8_t stir_target_inited = 0U;
	uint8_t last_fire_btn = 0U;
	uint8_t last_auto_shoot_cmd = 0U;
	uint16_t fire_hold_ticks = 0U;
	uint16_t fire_burst_ticks = 0U;
	uint8_t auto_fire_active = 0U;
	uint8_t auto_burst_pending = 0U;
	uint16_t auto_burst_ticks = 0U;
	uint8_t stir_jam_latched = 0U;
	uint16_t stir_jam_detect_ticks = 0U;
	uint16_t stir_jam_recover_ticks = 0U;
	uint16_t stir_jam_cooldown_ticks = 0U;
	stir_jam_state_e stir_jam_state = STIR_JAM_IDLE;
	(void)argument;

	for (;;)
	{
		uint8_t fire_cmd = 0U;
		uint8_t auto_shoot_cmd = 0U;
		uint8_t reverse_cmd;
		uint8_t heat_block = 0U;
		uint8_t fw_offline_block = 0U;
		uint8_t feed_block = 0U;
		int16_t stir_current = 0;
		int16_t stir_vel = 0;

		if (shoot_l == NULL || shoot_r == NULL || stir_m == NULL)
		{
			osDelay(2);
			continue;
		}

		if (robot_ctrl.shoot_mode == SHOOT_READY)
		{
			shoot_l->set_target(shoot_l, 1, SHOOT_FW_SPEED);
			shoot_r->set_target(shoot_r, 1, -SHOOT_FW_SPEED);
		}
		else
		{
			shoot_l->set_target(shoot_l, 1, 0);
			shoot_r->set_target(shoot_r, 1, 0);
		}

		stir_m->get_status(stir_m, "POS_SUM", &stir_pos_sum);
		stir_m->get_status(stir_m, "CURRENT", &stir_current);
		stir_m->get_status(stir_m, "VEL", &stir_vel);
		robot_ctrl.motors_info.m2006_trigger.current = stir_current;
		if (stir_target_inited == 0U)
		{
			stir_target_sum = stir_pos_sum;
			stir_target_inited = 1U;
		}

		/* C 档用于强制停转，不再作为拨弹反转触发。 */
		reverse_cmd = robot_ctrl.rc->vt13.mouse_vt13.press_m;
		heat_block = (robot_ctrl.game_info.shooter_17mm_barrel_heat >= SHOOT_HEAT_LIMIT_17MM) ? 1U : 0U;
		fw_offline_block = (robot_ctrl.motors_info.m3508_shoot_l.online == 0U ||
							robot_ctrl.motors_info.m3508_shoot_r.online == 0U) ? 1U : 0U;
		feed_block = (heat_block != 0U || fw_offline_block != 0U) ? 1U : 0U;

		/* 过热或摩擦轮掉线时只禁止拨弹：摩擦轮照常转��拨弹目标锁定当前位置 */
		if (feed_block != 0U)
		{
			stir_target_sum = stir_pos_sum;
			last_fire_btn = 0U;
			last_auto_shoot_cmd = 0U;
			fire_hold_ticks = 0U;
			fire_burst_ticks = 0U;
			auto_fire_active = 0U;
			auto_burst_pending = 0U;
			auto_burst_ticks = 0U;
			stir_jam_latched = 0U;
			stir_jam_detect_ticks = 0U;
			stir_jam_recover_ticks = 0U;
			stir_jam_cooldown_ticks = 0U;
			stir_jam_state = STIR_JAM_IDLE;
		}

		if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE)
		{
			fire_cmd = (robot_ctrl.rc->vt13.mouse_vt13.press_l || robot_ctrl.rc->vt13.rc_vt13.trigger)
					 && (robot_ctrl.shoot_mode == SHOOT_READY)
					 && (feed_block == 0U);
		}
		else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO)
		{
			/* Auto-aim: shoot上升沿触发三连发，不跟随高电平持续发射。 */
			auto_shoot_cmd = (robot_ctrl.target_info.shoot == 1U)
						 && (robot_ctrl.shoot_mode == SHOOT_READY)
						 && (feed_block == 0U);

			if (auto_shoot_cmd && (last_auto_shoot_cmd == 0U))
			{
				auto_burst_pending = AUTO_BURST_SHOTS;
				auto_burst_ticks = 0U;
			}
			last_auto_shoot_cmd = auto_shoot_cmd;
		}
		else
		{
			last_auto_shoot_cmd = 0U;
			auto_burst_pending = 0U;
			auto_burst_ticks = 0U;
		}

		if (reverse_cmd)
		{
			/* para_num=2: speed override mode */
			stir_m->set_target(stir_m, 2, STIR_REVERSE_SPEED, 1.0);
			stir_target_sum = stir_pos_sum;
			last_fire_btn = 0U;
			last_auto_shoot_cmd = 0U;
			fire_hold_ticks = 0U;
			fire_burst_ticks = 0U;
			auto_fire_active = 0U;
			auto_burst_pending = 0U;
			auto_burst_ticks = 0U;
			stir_jam_latched = 0U;
			stir_jam_detect_ticks = 0U;
			stir_jam_recover_ticks = 0U;
			stir_jam_cooldown_ticks = 0U;
			stir_jam_state = STIR_JAM_IDLE;
		}
		else
		{
			if (stir_jam_state == STIR_JAM_RECOVER)
			{
				stir_m->set_target(stir_m, 2, STIR_REVERSE_SPEED, 1.0);
				stir_target_sum = stir_pos_sum;
				last_fire_btn = 0U;
				last_auto_shoot_cmd = 0U;
				fire_hold_ticks = 0U;
				fire_burst_ticks = 0U;
				auto_fire_active = 0U;
				auto_burst_pending = 0U;
				auto_burst_ticks = 0U;

				if (stir_jam_recover_ticks < STIR_JAM_REVERSE_TICKS)
				{
					stir_jam_recover_ticks++;
				}
				else
				{
					stir_jam_state = STIR_JAM_COOLDOWN;
					stir_jam_cooldown_ticks = 0U;
				}

				osDelay(2);
				continue;
			}

			if (stir_jam_state == STIR_JAM_COOLDOWN)
			{
				fire_cmd = 0U;
				stir_target_sum = stir_pos_sum;
				last_fire_btn = 0U;
				last_auto_shoot_cmd = 0U;
				fire_hold_ticks = 0U;
				fire_burst_ticks = 0U;
				auto_fire_active = 0U;
				auto_burst_pending = 0U;
				auto_burst_ticks = 0U;

				if (stir_jam_cooldown_ticks < STIR_JAM_COOLDOWN_TICKS)
				{
					stir_jam_cooldown_ticks++;
				}
				else
				{
					stir_jam_state = STIR_JAM_IDLE;
					stir_jam_latched = 0U;
					stir_jam_detect_ticks = 0U;
				}
			}

			if ((stir_jam_state == STIR_JAM_IDLE) && fire_cmd &&
				(robot_ctrl.motors_info.m2006_trigger.online == 1U))
			{
				int32_t abs_current = (stir_current >= 0) ? (int32_t)stir_current : -(int32_t)stir_current;
				int32_t abs_vel = (stir_vel >= 0) ? (int32_t)stir_vel : -(int32_t)stir_vel;

				if ((abs_current >= STIR_JAM_CURRENT_THRESH) && (abs_vel <= STIR_JAM_VEL_THRESH))
				{
					if (stir_jam_detect_ticks < STIR_JAM_DETECT_TICKS)
					{
						stir_jam_detect_ticks++;
					}
					else if (stir_jam_latched == 0U)
					{
						stir_jam_latched = 1U;
						stir_jam_state = STIR_JAM_RECOVER;
						stir_jam_recover_ticks = 0U;
					}
				}
				else
				{
					stir_jam_detect_ticks = 0U;
				}
			}
			else if (stir_jam_state == STIR_JAM_IDLE)
			{
				stir_jam_detect_ticks = 0U;
			}

			if (robot_ctrl.gimbal_mode == GIMBAL_AUTO)
			{
				if (auto_burst_pending > 0U)
				{
					fire_cmd = 1U;
					if (auto_burst_ticks == 0U)
					{
						stir_target_sum += STIR_STEP_TICKS;
						auto_burst_pending--;
						auto_burst_ticks = AUTO_BURST_TICKS;
					}
					else
					{
						auto_burst_ticks--;
					}
				}
				else
				{
					int32_t pos_err = stir_target_sum - stir_pos_sum;
					int32_t abs_err = (pos_err >= 0) ? pos_err : -pos_err;
					fire_cmd = (abs_err > STIR_TARGET_HOLD_TOL) ? 1U : 0U;
				}

				if (!fire_cmd)
				{
					stir_target_sum = stir_pos_sum;
				}

				last_fire_btn = 0U;
				fire_hold_ticks = 0U;
				fire_burst_ticks = 0U;
				auto_fire_active = 0U;
			}
			else
			{
				if (fire_cmd && (last_fire_btn == 0U))
				{
					/* 上升沿：先打一发 */
					stir_target_sum += STIR_STEP_TICKS;
					fire_hold_ticks = 0U;
					fire_burst_ticks = 0U;
					auto_fire_active = 0U;
				}

				if (fire_cmd)
				{
					if (fire_hold_ticks < FIRE_HOLD_START_TICKS)
					{
						fire_hold_ticks++;
					}
					else
					{
						auto_fire_active = 1U;
					}

					if (auto_fire_active)
					{
						if (fire_burst_ticks >= FIRE_BURST_TICKS)
						{
							stir_target_sum += STIR_STEP_TICKS;
							fire_burst_ticks = 0U;
						}
						else
						{
							fire_burst_ticks++;
						}
					}
				}
				else
				{
					/* 非按下状态每帧锁位：松手后立即停止拨弹，不执行历史积压目标 */
					stir_target_sum = stir_pos_sum;
					last_fire_btn = 0U;
					fire_hold_ticks = 0U;
					fire_burst_ticks = 0U;
					auto_fire_active = 0U;
				}
			}

			/* para_num=1: position mode target */
			stir_m->set_target(stir_m, 1, (double)stir_target_sum);
			if (fire_cmd) {
				last_fire_btn = 1U;
			}
		}

		osDelay(2);
	}
}
