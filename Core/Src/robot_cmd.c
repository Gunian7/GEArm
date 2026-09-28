#include "robot.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>
#include "robot_cmd.h"
#include "Emm_V5.h"

static int robot_soft_reset_handle(float *param);
static int robot_rel_rotate_handle(float *param);
static int robot_auto_handle(float *param);
static int robot_abs_rotate_handle(float *param);

void robot_mqtt_handle(struct robot_cmd *cmd)
{
	float param[6] = {0};
	int strlen = 0;
	int type = 0;
	
	LOG("robot mqtt cmd: %s\n", cmd->cmd);

	// [MCU][TYPE][ARG0-5]
	int result = sscanf(cmd->cmd, "+MQTTSUBRECV:0,\"arm/change\",%d,[MCU][%d][%f %f %f %f %f %f]", &strlen, &type,
			&param[0], &param[1], &param[2],
			&param[3], &param[4], &param[5]);
	
	if (result < 8) { // 解析失败
		return;
	}
	
	switch (type)
	{
		case ROBOT_JOINT_ABS_ROTATE:
			robot_abs_rotate_handle(param);
			break;
		
		case ROBOT_AUTO_EVENT:
			robot_auto_handle(param);
			break;
		
		case ROBOT_JOINTS_SYNC_EVENT:
			robot_auto_handle(param);
			break;
		
		default:
			break;
	}
}

static int robot_remote_enable_handle(float *param)
{
	robot_soft_reset_handle(param);	/* 复位 */
	ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	return robot_send_remote_event();
}

static int robot_remote_disable_handle(float *param)
{
	(void)param;
	ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	robot_soft_reset_handle(param);	/* 复位 */
	return pdPASS;	
}

static int robot_rel_rotate_handle(float *param)
{
	uint32_t joint_id = (uint32_t)param[0];
	return robot_send_rel_rotate_event(joint_id, param[1]);
}

static int robot_abs_rotate_handle(float *param)
{
	uint32_t joint_id = (uint32_t)param[0];
	return robot_send_abs_rotate_event(joint_id, param[1]);
}

static int robot_auto_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}

static int robot_joints_sync_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}

static int robot_hard_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(true);	
}

static int robot_soft_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(false);	
}

static int robot_time_func_handle(float *param)
{
	return robot_send_time_func_event(param[0] * 1000);
}

static int robot_remote_event_handle(float *param)
{
	if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		return pdPASS;
	}

	float vx = -param[0] * ROBOT_REMOTE_MAX_VELOCITY;
	float vy = param[1] * ROBOT_REMOTE_MAX_VELOCITY;
	float vz = (param[4] - param[5]) / 2 * ROBOT_REMOTE_MAX_VELOCITY;
	float rx = -param[3] * ROBOT_REMOTE_MAX_RPM;
	float ry = param[2] * ROBOT_REMOTE_MAX_RPM;
	
	taskENTER_CRITICAL();
	g_remote_control.vx = vx;
	g_remote_control.vy = vy;
	g_remote_control.vz = vz;
	g_remote_control.rx = rx;
	g_remote_control.ry = ry;
	taskEXIT_CRITICAL();

	return pdPASS;
}

static int robot_zero_handle(float *param)
{
	(void)param;
	LOG("robot reset zero.\n");
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Reset_CurPos_To_Zero(i + 1);
		vTaskDelay(10);
	}
	return pdPASS;
}

static struct robot_cmd_info robot_uart1_cmd_table[] = {
	{"remote_event", robot_remote_event_handle},
	{"remote_enable", robot_remote_enable_handle},
	{"remote_disable", robot_remote_disable_handle},
	{"rel_rotate", robot_rel_rotate_handle},
	{"auto", robot_auto_handle},
	{"hard_reset", robot_hard_reset_handle},
	{"soft_reset", robot_soft_reset_handle},
	{"zero", robot_zero_handle},
	// {"time_func", robot_time_func_handle},
	{NULL, NULL},
};

void robot_uart1_handle(struct robot_cmd *rb_cmd)
{
	static char event_type[20] = {0};
	float param[6] = {0};
	char *cmd = rb_cmd->cmd;
	int ret;

	ret = sscanf(cmd, "%19s %f %f %f %f %f %f", event_type, &param[0], &param[1], &param[2], 
		&param[3], &param[4], &param[5]);
	if (ret < 1) { // 解析失败
        LOG("event_type parse error: %s\n", cmd);
        return;
    }

	for (int i = 0; robot_uart1_cmd_table[i].event_type != NULL; i++) {
		if (strcmp(event_type, robot_uart1_cmd_table[i].event_type) == 0) {
			ret = robot_uart1_cmd_table[i].cmd_func(param);
			if (ret != pdPASS) {
				LOG("[ERROR] [jid:%d] event_type:%s param:%.2f %.2f %.2f\n", event_type, param[0], param[1], param[2]);
				return;
			}
			return;
		}
	}

	LOG("uart cmd parse error: %s\n", cmd);
	return;
}

