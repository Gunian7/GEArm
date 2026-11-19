#include "robot.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>
#include "robot_cmd.h"
#include "Emm_V5.h"
#include "usbd_cdc_if.h"   /* usb_cdc_send_text()：把解析错误直接报到 Web 控制台 */

static int robot_soft_reset_handle(float *param);
static int robot_rel_rotate_handle(float *param);
static int robot_auto_handle(float *param);
static int robot_abs_rotate_handle(float *param);
static int robot_estop_handle(float *param);
static int robot_estop_clear_handle(float *param);

/* 只读诊断：查询驱动器状态（3A/35/36/1F），不产生任何运动。
 * 用法：qstat      -> 查地址 1
 *       qstat 2    -> 查地址 2 */
static int robot_qstat_handle(float *param);
static int robot_jstat_handle(float *param);

/* 重新探测总线成员（接好新电机后发 probe） */
static int robot_probe_handle(float *param);

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

/* 急停：立即停所有电机并阻塞后续动作。见 robot.c 急停小节的设计说明。 */
static int robot_estop_handle(float *param)
{
	(void)param;
	robot_estop_trigger();
	return pdPASS;
}

/* 解除急停。必须人工显式调用，固件不会自动恢复。 */
static int robot_estop_clear_handle(float *param)
{
	(void)param;
	robot_estop_clear();
	return pdPASS;
}

/* 只读诊断：问驱动"你到底什么状态"。
 * 电机不动而命令明明发出去了（g_dbg_can_tx_ok 在涨）时，这是唯一能区分
 * "没使能 / 堵转 / 位置没变" 的手段。不产生运动。 */
static int robot_qstat_handle(float *param)
{
	uint8_t addr = 1u;

	if (param != NULL && param[0] >= 1.0f) {
		addr = (uint8_t)param[0];
	}
	if (addr < 1u) { addr = 1u; }
	if (addr > ROBOT_MAX_JOINT_NUM) { addr = ROBOT_MAX_JOINT_NUM; }

	robot_diag_query_drive(addr);
	return pdPASS;
}

/* jstat：只读诊断，把 6 个轴的当前角度、软件限位范围、减速比打到 USB。
 * 用来验证"开机位置同步"和"软件限位"是否真的生效：
 *   开机后第一次 jstat -> cur 应该是电机实际角度，而不是 g_joints_init 的硬编码值
 *   发一条越界 rel_rotate 后再 jstat -> cur 应该停在 limit 的上/下限上 */
static int robot_jstat_handle(float *param)
{
	(void)param;
	robot_diag_joints();
	return pdPASS;
}

/* 重新探测总线 + 给在场的电机使能。
 * 用在一颗一颗接线的阶段：新电机接好、配好 ID 之后发 probe，
 * 它就会进入在场名单，之后的运动指令才发得出去。 */
static int robot_probe_handle(float *param)
{
	char msg[64];

	(void)param;
	robot_motor_enable_all();       /* 内部先探测，只给在场的使能 */

	snprintf(msg, sizeof(msg), "probe: present mask 0x%02X\r\n",
	         (unsigned)g_can_present_mask);
	usb_cdc_send_text(msg);
	return pdPASS;
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
	{"remote_event",   robot_remote_event_handle,   6},
	{"remote_enable",  robot_remote_enable_handle,  0},
	{"remote_disable", robot_remote_disable_handle, 0},
	{"rel_rotate",     robot_rel_rotate_handle,     2},   /* 关节号 + 角度 */
	{"auto",           robot_auto_handle,           6},   /* x y z rx ry rz */
	{"hard_reset",     robot_hard_reset_handle,     0},
	{"estop",          robot_estop_handle,          0},
	{"estop_clear",    robot_estop_clear_handle,    0},
	{"soft_reset",     robot_soft_reset_handle,     0},
	{"zero",           robot_zero_handle,           0},
	{"qstat",          robot_qstat_handle,          0},
	{"jstat",          robot_jstat_handle,          0},   /* 只读诊断：6 轴角度/软件限位/减速比 */
	{"probe",          robot_probe_handle,          0},   /* 重新探测总线成员并使能在场的电机 */   /* 只读诊断：驱动 3A/35/36/3C/37/1F */
	// {"time_func", robot_time_func_handle},
	{NULL, NULL, 0},
};

extern volatile uint32_t g_dbg_cmd_parsed;

/* ---------------------------------------------------------------------------
 * 浮点解析自检（只跑一次，结果存 g_dbg_scanf_float_ok 供 J-Link 读）
 *
 * newlib-nano 的 sscanf 默认**不支持 %f**：必须靠链接期 `-u _scanf_float`
 * 把浮点解析代码拉进来。漏了的话 sscanf 不报错，只是静默地少转换几项、
 * 把参数留在 0 —— 表现为"命令收到了、也执行了，但角度永远是 0"。
 *
 * 曾经的真实后果：`rel_rotate 0 5` 被解析成 `rel_rotate 0 0`，steps 算成 0，
 * 电机收到"移动 0 个脉冲"后回 FD 02（成功）却一动不动，导致排查方向被
 * 错误地引向 CAN 帧格式、波特率、驱动器菜单，绕了一大圈。
 * --------------------------------------------------------------------------- */
volatile uint32_t g_dbg_scanf_float_ok = 0xFFFFFFFFu;   /* 0xFF.. = 还没测 */
volatile float    g_dbg_selftest_angle = -1.0f;         /* 自检解析出的角度，应为 5.0 */

/* 开机自检：用和真实命令**完全相同的格式串**解析一遍，结果留在两个全局里。
 *
 * 这样验证"浮点 scanf 到底行不行"完全不需要 USB：烧录后 J-Link 直接读
 *   g_dbg_scanf_float_ok  -> 1 = 好，0 = 链接漏了 -u _scanf_float
 *   g_dbg_selftest_angle  -> 5.0 = 好，0.0 = %f 没生效
 * USB 通道本身出问题的时候，这是唯一可靠的判据。 */
void robot_cmd_selftest(void)
{
	char evt[20] = {0};
	float p[6] = {0};
	int n = sscanf("rel_rotate 0 5", "%19s %f %f %f %f %f %f",
	               evt, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]);

	g_dbg_selftest_angle = p[1];
	g_dbg_scanf_float_ok = (uint32_t)((n == 3 && p[1] == 5.0f) ? 1u : 0u);

	/* 只用 %d，避免"用坏的 printf 去证明 printf 是好的" */
	LOG("selftest sscanf: n=%d p0=%d p1=%d (expect 3,0,5)\n", n, (int)p[0], (int)p[1]);
}

static int scanf_float_supported(void)
{
	if (g_dbg_scanf_float_ok == 0xFFFFFFFFu) {
		robot_cmd_selftest();
	}
	if (g_dbg_scanf_float_ok != 1u) {
		LOG("FATAL: sscanf %%f unsupported — relink with -u _scanf_float\n");
	}
	return (int)g_dbg_scanf_float_ok;
}

static void parse_error_report(const char *cmd, const char *why, int need, int got)
{
	char msg[96];
	/* 只用到 %s/%d，nano printf 支持；这里刻意不碰 %f */
	snprintf(msg, sizeof(msg), "!parse %s: %s (need %d, got %d)\r\n", cmd, why, need, got);
	usb_cdc_send_text(msg);
	LOG("parse reject: %s\n", msg);
}

void robot_uart1_handle(struct robot_cmd *rb_cmd)
{
	static char event_type[20] = {0};
	float param[6] = {0};
	char *cmd = rb_cmd->cmd;
	int ret;

	ret = sscanf(cmd, "%19s %f %f %f %f %f %f", event_type, &param[0], &param[1], &param[2], 
		&param[3], &param[4], &param[5]);
	if (ret < 1) { // 解析失败
        parse_error_report(cmd, "unparsable", 1, ret);
        return;
    }

	for (int i = 0; robot_uart1_cmd_table[i].event_type != NULL; i++) {
		if (strcmp(event_type, robot_uart1_cmd_table[i].event_type) == 0) {
			const struct robot_cmd_info *info = &robot_uart1_cmd_table[i];
			int need = 1 + (int)info->min_args;

			/* 参数个数守卫：见 robot_cmd.h 里 min_args 的说明。
			 * 少了参数宁可拒掉，也不能带着 param=0 继续往下发运动指令。 */
			if (ret < need) {
				parse_error_report(event_type, "missing/zero args", info->min_args, ret - 1);
				return;
			}
			/* 需要数值参数的命令，顺带确认 %f 真的被支持 */
			if (info->min_args > 0 && !scanf_float_supported()) {
				parse_error_report(event_type, "scanf float unsupported, relink with -u _scanf_float",
				                   info->min_args, ret - 1);
				return;
			}
			g_dbg_cmd_parsed++;
			ret = info->cmd_func(param);
			if (ret != pdPASS) {
				/* 原来的格式串把 %s 写成了 %d、还把 float 传给 %d，已修正 */
				LOG("[ERROR] event_type:%s param:%d %d %d\n",
				    event_type, (int)param[0], (int)param[1], (int)param[2]);
				return;
			}
			return;
		}
	}

	parse_error_report(event_type, "unknown command", 0, 0);
	return;
}

