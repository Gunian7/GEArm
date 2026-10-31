/**
  ******************************************************************************
  * @file    robot.c
  * @brief   This file provides code for the robot control frame.
  ******************************************************************************
  */

#include "robot.h"
#include "task.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "Emm_V5.h"
#include "usart.h"
#include "main.h"
#include "robot_kinematics.h"
#include <stdlib.h>
#include "robot_cmd.h"
#include "esp8266_mqtt.h"
#include "fdcan.h"

HAL_StatusTypeDef CAN_Send_Msg(uint32_t StdId, uint8_t *pData, uint8_t Len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    // 1. 配置 FDCAN 发送头
    TxHeader.Identifier = StdId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = (Len > 8) ? FDCAN_DLC_BYTES_8 : ((uint32_t)Len << 16); 
    // 注意: H7 的 DLC 宏定义很怪 (FDCAN_DLC_BYTES_8 = 0x00080000U)，
    // 这里简单处理：如果 Len <= 8，直接左移 16 位通常能对应上 (比如 8<<16 = 0x80000)，
    // 但稳妥起见，最好用 switch-case：
    switch(Len) {
        case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
        default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 经典 CAN 不加速
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 2. 发送
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
    {
        return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pData);
    }
    return HAL_BUSY;
}

struct robot g_robot;       /* robot实例 */

/* 机械臂各关节DH参数, 需手动设置 */
const float D_H[6][4] = {{0,        0,          0,          M_PI/2},
                         {0,        M_PI/2,      0,         M_PI/2},
                         {200,      M_PI,        0,         -M_PI/2},
                         {47.63,    -M_PI/2,     -184.5,    0},
                         {0,        M_PI/2,      0,         M_PI/2},
                         {0,        M_PI/2,        0,         0}};

/* 机械臂复位状态下T0_6矩阵, 需手动设置 */
const float T_0_6_reset[4][4] = {
    {0, -1, 0, 0},
    {0, 0, -1, -47.63},
    {1, 0, 0, 15.5},
    {0, 0, 0, 1},
};

/* 各关节旋转的权重, 需手动设置 */
const float joint_weight[ROBOT_MAX_JOINT_NUM] = {5, 3, 3, 1, 1, 1};

/* 机械臂各关节初始状态, 需手动设置 */
static struct joint g_joints_init[ROBOT_MAX_JOINT_NUM] = {
    {90,    MOTOR_DIR_CCW,   50,     JOINT_LIMIT_1_GPIO_Port,    JOINT_LIMIT_1_Pin,      0,      360,    DIR_NEGATIVE},  /* 关节1 */
    {90,    MOTOR_DIR_CW,  50.89,  JOINT_LIMIT_2_GPIO_Port,    JOINT_LIMIT_2_Pin,       90,     180,    DIR_NEGATIVE},  /* 关节2 */
    {-90,   MOTOR_DIR_CW,   50.89,  JOINT_LIMIT_3_GPIO_Port,    JOINT_LIMIT_3_Pin,      -90,    90 ,    DIR_NEGATIVE},  /* 关节3 */
    {0,     MOTOR_DIR_CW,  51,     JOINT_LIMIT_4_GPIO_Port,    JOINT_LIMIT_4_Pin,       -90,   90,    DIR_NEGATIVE},  /* 关节4 */
    {90,    MOTOR_DIR_CCW,   26.85,  JOINT_LIMIT_5_GPIO_Port,    JOINT_LIMIT_5_Pin,     0,      90,     DIR_POSITIVE},  /* 关节5 */
    {0,     MOTOR_DIR_CW,   51,     JOINT_LIMIT_6_GPIO_Port,    JOINT_LIMIT_6_Pin,      0,      360,    DIR_NEGATIVE},  /* 关节6 */
};

volatile struct robot_remote_control g_remote_control = {0};

static struct position *robot_path_interpolation_linear(struct position *target, int *size);
static int robot_update_current_angle(uint8_t joint_id);
static void robot_joint_stop(uint8_t joint_id);
static int time_func_circle(uint32_t time_ms, struct position *pos);
static int robot_pid_run(struct position *path, int path_size, float *result);
static void robot_pid_one_period(float *target_angle, float *intg_error, float *pre_error, float *total_error, int joint_num);
static int robot_pid_remote(void);
static int robot_mqtt_joints_sync(void);
static void robot_joint_stop_from_isr(uint8_t joint_id);

static robot_time_func g_robot_time_func = time_func_circle; /* 时间函数 */

/* ---------------------------------------------------------------------------
 * 诊断计数器：定位命令链断点。
 *
 * 链路：USB CDC 收一行 → 入 cmd_queue → robot_cmd_service 解析 →
 *       入 event_queue → robot_control_task → robot_joint_rotate_to
 *
 * 之前的现象：每条命令都收到 "+OK" 回显，但 joints[].current_angle 不变。
 * 回显在 CDC_Receive_HS 里是无条件发的，只证明"USB 中断跑过"，
 * 不证明命令进了队列。用 J-Link 读这几个全局值就能一次定位卡在哪一级：
 *   g_dbg_usb_rx        涨、g_dbg_cmd_queued 不涨 → ISR 入队失败
 *   g_dbg_cmd_queued    涨、g_dbg_cmd_parsed  不涨 → cmd_service 没跑
 *   g_dbg_cmd_parsed    涨、g_dbg_event_queued 不涨 → 分发被拒
 *   g_dbg_event_queued  涨、g_dbg_rotate_called 不涨 → control_task 没跑
 *   g_dbg_rotate_called 涨但电机不动 → CAN/电机侧问题
 * ------------------------------------------------------------------------- */
volatile uint32_t g_dbg_cmd_queued = 0;
volatile uint32_t g_dbg_cmd_parsed = 0;
volatile uint32_t g_dbg_event_queued = 0;
volatile uint32_t g_dbg_rotate_called = 0;
volatile uint32_t g_dbg_rotate_reject = 0;


/* -------------------------------------------------------------------------
 * 急停（2026-10-28 新增）
 *
 * 为什么必须有：机械臂持续运动时如果指令发错（逆运动解算出错误关节角、Web 端
 * 误操作、插值路径异常），机械臂会以默认 10rpm 持续朝错误方向转，可能撞坏结构
 * 或拉断线。需要一个人能一键、快速、可靠停下来的手段。
 *
 * 设计要点：
 * 1) 必须走非阻塞路径。CAN_Send_Msg() 只是往 FDCAN TX FIFO 入队
 *    （HAL_FDCAN_GetTxFifoFreeLevel + AddMessageToTxFifoQ，满了返回 HAL_BUSY），
 *    不等待电机应答，所以 6 个电机的停止命令可以一次性全部发出（微秒级）。
 *    绝不能用 robot_joint_stop() —— 它 vTaskSuspendAll() 挂起调度器后循环发
 *    停止命令并等待 CAN 应答、超时才返回，连续调 6 次会让急停本身阻塞最多
 *    6 × ROBOT_CAN_TIMEOUT，完全违背"急停要快"的初衷。
 * 2) 同时置 g_estop 标志，让 robot_pid_run() / robot_control_task 中止。
 *    只给电机发停止是不够的 —— robot_pid_run() 的双层循环还在持续往下发位置
 *    指令，电机会被重新驱动，等于没停。
 * 3) 急停后必须显式 estop_clear 才恢复，不会自动解除，避免危险状态下自己恢复。
 * 4) 停电机用 Emm_V5_Stop_Now（减速并保持位置），不用 Emm_V5_En_Control 掉使能
 *    —— 掉使能会失去保持力矩，机械臂会在重力下垮下去，那更危险。
 *
 * 延迟：Web 按钮 → HTTP → bridge → USB → 命令队列 → 解析 → CAN，约 10~20ms。
 * 对"机械臂发疯"这个场景够用，但不是硬实时；真要微秒级得走独立硬件回路。
 * ------------------------------------------------------------------------- */
static volatile bool g_estop = false;

void robot_estop_trigger(void)
{
    g_estop = true;
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        Emm_V5_Stop_Now(i + 1, false);   /* 非阻塞：只入队，不等待应答 */
        g_robot.joints[i].velocity = 0;
    }
    LOG("ESTOP: all %d joints stopped, motion blocked\n", ROBOT_MAX_JOINT_NUM);
}

void robot_estop_clear(void)
{
    g_estop = false;
    LOG("ESTOP cleared, motion allowed again\n");
}

int robot_estop_active(void)
{
    return g_estop ? 1 : 0;
}

static void robot_joint_limit_post_handle(uint8_t joint_id)
{
	vTaskDelay(200); // 延时200ms等待限位开关稳定
	taskENTER_CRITICAL();
	// 清除限位状态位
	ROBOT_STATUS_CLEAR(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);
	taskEXIT_CRITICAL();
}

uint32_t robot_joint_veloccity_to(uint32_t joint_id, float velocity,\
    uint8_t acceleration)
{
	if (joint_id >= ROBOT_MAX_JOINT_NUM) {
        return 1;
    }

	int start_tick = HAL_GetTick();
    struct joint *joint = &g_robot.joints[joint_id];
	
	uint8_t dir = (velocity > 0) ? joint->postive_direction : !(joint->postive_direction);
	/* 注意：原代码在这里 ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_LIMIT_ENABLE)
	 * 会把限位保护关掉，导致速度模式下限位开关形同虚设。限位保护必须始终有效，
	 * 因此不在此清限位使能。 */
	uint32_t addr = joint_id + 1; // 各关节CAN地址从1开始

	// 计算电机速度，单位：rpm. 驱动器会将_velocity/10作为真实速度，从而实现0.1RPM精度控制，因此计算时我们需要提前乘以10
	joint->velocity = velocity;
	uint16_t _velocity = (uint16_t)fabs(velocity * 600 * joint->reduction_ratio / 360);
	// 关任务调度
	vTaskSuspendAll();
	can.rxFrameFlag = false;
	while(can.rxFrameFlag == false) {
		if ((HAL_GetTick() - start_tick) > ROBOT_CAN_TIMEOUT) {
			LOG("ERROR: CAN timeout:%d\n", joint_id);
			xTaskResumeAll();
			return 1;
		}
		Emm_V5_Vel_Control(addr, dir, _velocity, acceleration, false);
		HAL_Delay(1);
	}
	xTaskResumeAll();
	return 0;
}

/* 控制关节运动 */
static uint32_t robot_joint_rotate_to(uint32_t joint_id, enum dir dir, float angle, float velocity,\
    uint32_t acceleration, bool absolute)
{
	g_dbg_rotate_called++;

	if (joint_id >= ROBOT_MAX_JOINT_NUM) {
		LOG("ERROR: joint_id is out of range");
		g_dbg_rotate_reject++;
        return 1;
    }

	if (velocity < 0) {
		LOG("ERROR: velocity is negative");
		g_dbg_rotate_reject++;
		return 1;
	}

   float rel_angle = 0;
   uint8_t _dir;
   struct joint *joint = &g_robot.joints[joint_id];

    if (absolute) {
        rel_angle = angle - joint->current_angle;
		
		if (fabs(rel_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
			return 0;
		}

		_dir = (dir == DIR_POSITIVE)? joint->postive_direction :!(joint->postive_direction);

		if ((rel_angle < 0) && (dir != DIR_NEGATIVE)) {
			rel_angle = 360 - fabs(rel_angle);
		} else if ((rel_angle > 0) && (dir != DIR_POSITIVE)) {
			rel_angle = 360 - fabs(rel_angle);
		}

		LOG_FROM_ISR("id:%d current:%f, target:%f, rel:%f dir:%d\n", joint_id, joint->current_angle, angle, rel_angle, dir);
		joint->current_angle = angle;
    } else {
        /* 相对角度 */
        rel_angle = angle;
        joint->current_angle += (dir == DIR_POSITIVE) ? angle : -angle;
		_dir = (dir == DIR_POSITIVE) ? joint->postive_direction : !(joint->postive_direction);
		if (rel_angle < 0) {
			_dir = !_dir;
		}
    }

    /* 计算旋转步数 */
    // 16细分下发送3200个脉冲电机转一圈
    uint32_t steps = (uint32_t)fabs(round(rel_angle * joint->reduction_ratio * 3200 / 360));
	uint16_t _velocity = (uint16_t)fabs(velocity * 600 * joint->reduction_ratio / 360.0);
    uint32_t addr = joint_id + 1; // 各关节CAN地址从1开始
    ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_READY);
    Emm_V5_Pos_Control(addr, _dir, _velocity, (uint8_t)acceleration, steps, false, false);
    return 0;
}

static void robot_joint_limit_happend(uint8_t joint_id)
{
	if (g_robot.event_queue == NULL) {
        return;
    }

    // 限位开关未使能
    if (!ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_ENABLE)) {
        return;
    }

	// 防止按键抖动，重复发事件
    if (ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED)) {
        return;
	}

	// 该函数仅会在中断中调用，且不存在多个中断同时并发设置一个关节状态位的情况，因此无需使用临界区保护
	robot_joint_stop_from_isr(joint_id);
	ROBOT_STATUS_SET(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);

	struct robot_event event = {0};
	event.type = ROBOT_LIMIT_SWITCH_EVENT;
	event.joint_id = joint_id;
	BaseType_t xHigherPriorityTaskWoken;
    xQueueSendToBackFromISR(g_robot.event_queue, &event, &xHigherPriorityTaskWoken);
}

static void robot_joint_limit_set_input(uint8_t joint_id)
{
	// 修改为输入模式
    HAL_GPIO_DeInit(g_robot.joints[joint_id].limit_gpio_port, g_robot.joints[joint_id].limit_gpio_pin);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = g_robot.joints[joint_id].limit_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(g_robot.joints[joint_id].limit_gpio_port, &GPIO_InitStruct);
}

static void robot_joint_limit_set_irq(uint8_t joint_id)
{
	/* 常闭 NC 开关（沿用原项目 KW11-3Z 接法）：COM→GND，NC→GPIO，NO 悬空。
	 * 未按压时 NC 闭合，GPIO 被拉到 GND（低）；按压时 NC 断开，内部上拉把
	 * GPIO 拉高（高）。因此 GPIO_PIN_SET = 已触发（按压）。
	 *
	 * 2026-10-28 改为轮询检测，此函数退化成纯输入配置。
	 *
	 * 原来这里是 GPIO_MODE_IT_RISING，必须删掉，原因是硬件限制：
	 * SYSCFG_EXTICR 给每条 EXTI 线只有 4 bit 的端口选择字段（PA=0..PH=7），
	 * 一条线只能选一个端口。而本方案里 J2=PA02 与 J5=PD02 同为引脚号 2、
	 * 同一条 EXTI2 线 —— 已实测证明：后调用 HAL_GPIO_Init 的那个会把前一个的
	 * 路由静默覆盖，被覆盖的那个中断永远不触发（读 EXTICR1 可见 0x300→0x000）。
	 * 这不是回调里能绕过的，根因在硬件。
	 *
	 * 轮询读 HAL_GPIO_ReadPin(端口, 引脚)，端口+引脚组合唯一，同号引脚也分得清。
	 * 边沿检测见 robot_limit_poll()。
	 *
	 * 保留本函数名与调用点不变，是为了让 robot_joint_reset() 的流程一行不改。 */
    robot_joint_limit_set_input(joint_id);
}

/* -------------------------------------------------------------------------
 * 限位轮询（2026-10-28 新增）
 *
 * 由 main.c 的 HAL_TIM_PeriodElapsedCallback() 在 TIM6 的 1kHz 中断里调用。
 * TIM6 本就是 HAL timebase（HAL_GetTick 用它），不额外占定时器、不占新中断，
 * 且优先级高，不会被 FreeRTOS 任务饿死 —— 比用任务 + vTaskDelay 更可靠
 * （vTaskDelay 的周期不保证，高优先级任务跑久了轮询会被无限推迟）。
 *
 * 成本：6 次 IDR 读 + 边沿比较 ≈ 50~100 周期；1kHz 下 ≈ 0.07% CPU（HCLK 137.5MHz），
 * 连中断进出算一起约 0.22%。可以忽略。
 *
 * 延迟：从 EXTI 的 ~50µs 变成最多 1ms。复位转速 10rpm(=60°/s) 下过冲 0.06°，
 * 机械上无感。
 *
 * 注意 J6(PE14) 没有接线（BOM 只有 5 个开关），悬空被上拉拉高会持续读到 SET，
 * 但 robot_joint_limit_happend() 开头的 ROBOT_STATUS_LIMIT_ENABLE 检查会让它
 * 直接返回，不会误停电机。
 * ------------------------------------------------------------------------- */
static GPIO_PinState g_limit_prev[ROBOT_MAX_JOINT_NUM];

void robot_limit_poll(void)
{
    /* 时序保护：TIM6 中断在 main() 的 HAL_Init() 里就被 HAL_InitTick() 使能了，
     * 而 g_robot.joints[] 要等到 MX_FREERTOS_Init() -> robot_init() 里才 memcpy 填充。
     * g_robot 是全局量零初始化，所以这段窗口里 limit_gpio_port 还是 NULL，
     * HAL_GPIO_ReadPin(NULL, pin) 会解引用空指针直接 HardFault。
     * 原来的 EXTI 方案没这个问题（NVIC 是 LimitSwitch_Init() 里才开的，晚于 HAL_Init），
     * 轮询挂在 TIM6 上才引入这个窗口，所以这里必须挡一下。
     * robot_init() 里的 memcpy 在队列创建之前，故端口指针非空即代表 joints 已就绪。 */
    if (g_robot.joints[0].limit_gpio_port == NULL) {
        return;
    }

    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
    {
        GPIO_PinState now = HAL_GPIO_ReadPin(g_robot.joints[i].limit_gpio_port,
                                             g_robot.joints[i].limit_gpio_pin);
        /* 上升沿 = 开关被按压（NC 断开，内部上拉拉高） */
        if ((now == GPIO_PIN_SET) && (g_limit_prev[i] != GPIO_PIN_SET))
        {
            robot_joint_limit_happend((uint8_t)i);
        }
        g_limit_prev[i] = now;
    }
}

static GPIO_PinState robot_get_limit_status(uint8_t joint_id)
{   
    // 读取限位开关状态
    GPIO_PinState state = HAL_GPIO_ReadPin(g_robot.joints[joint_id].limit_gpio_port, g_robot.joints[joint_id].limit_gpio_pin);
    return state;
}

static void robot_joint_reset(uint8_t joint_id)
{
	GPIO_PinState state;
    int reset_dir = g_robot.joints[joint_id].reset_dir;
    
    robot_joint_limit_set_input(joint_id); // 设置为输入模式
    state = robot_get_limit_status(joint_id); // 读取限位开关状态
    robot_joint_limit_set_irq(joint_id); // 设置为中断模式

    /* 常闭 NC 开关（沿用原项目 KW11-3Z 接法）：COM→GND，NC→GPIO，NO 悬空。
     * 未按压时 NC 闭合，GPIO 被拉到 GND（低）；按压时 NC 断开，内部上拉把
     * GPIO 拉高（高）。因此 GPIO_PIN_SET 表示限位开关已触发（按压）。 */
    if (state == GPIO_PIN_SET) { // 限位开关已触发, 无需复位
        LOG("joint %d limit switch already happend\n", joint_id);
        Emm_V5_Reset_CurPos_To_Zero(joint_id + 1); // 关节复位
        return;
    }

    robot_joint_rotate_to(joint_id, reset_dir, ROBOT_RESET_DEFAULT_ANGLE, ROBOT_RESET_DEFAULT_VELOCITY, 
                                ROBOT_RESET_DEFAULT_ACCELERATION, false);
    /* 逆时针旋转，直到检测到限位开关。必须有总超时，否则限位永不触发会死循环
     * 且电机持续转动到机械撞限位。200ms × 60 = 12s 上限。 */
    for (int i = 0; i < 60; i++) {
        if (ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED)) {
            vTaskDelay(ROBOT_CAN_DELAY);
            Emm_V5_Reset_CurPos_To_Zero(joint_id + 1); // 关节复位
            return;
        }
        vTaskDelay(200); // 等待关节转动
    }
    LOG("joint %d reset timeout, limit switch never triggered\n", joint_id);
    robot_joint_stop(joint_id);
    ROBOT_STATUS_CLEAR(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);
}

/* 复位所有关节
 * BOM 只有 5 个限位开关（KW11-3Z-A×3 + KW11-3Z-B×2），第 6 轴无开关，
 * 循环从 ROBOT_MAX_JOINT_NUM - 2 开始即跳过第 6 轴（index 5）。 */
static void robot_joint_hard_reset(void)
{   
    for (int i = ROBOT_MAX_JOINT_NUM - 2; i >= 0; i--) {
        ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_ENABLE);
		robot_joint_reset(i);
        vTaskDelay(100);
    }
    
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        g_robot.joints[i].current_angle = g_joints_init[i].current_angle;
    }
	g_robot.cur_pos.x = 0;
	g_robot.cur_pos.y = 0;
	g_robot.cur_pos.z = 0;
	robot_mqtt_joints_sync();
}

static int robot_angle_map(float angle, float min_angle, float max_angle, float *result)
{
    if (result == NULL) {
		return 1;
	}

    float tmp_angle = angle;
	
	// 防止实物抖动，导致错误判断
	if (fabs(angle - min_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = min_angle;
    } else if (fabs(angle - max_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = max_angle;
    }

	if (angle < min_angle) {
		tmp_angle += 360;
    } else if (angle > max_angle) {
    	tmp_angle -= 360;
    }

	// 防止实物抖动，导致错误判断
	if (fabs(tmp_angle - min_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = min_angle;
    } else if (fabs(tmp_angle - max_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = max_angle;
    }

	if ((tmp_angle < min_angle) || (tmp_angle > max_angle)) {
        return 1;
    }

    *result = tmp_angle;
    return 0;
}

static void robot_joint_soft_reset(void)
{
	float angle = 0;
	int ret = 0;
	for (int i = ROBOT_MAX_JOINT_NUM - 1; i >= 0; i--) {
		ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_ENABLE);
		int dir = DIR_POSITIVE;
		robot_update_current_angle(i);
		ret = robot_angle_map(g_robot.joints[i].current_angle, g_joints_init[i].min_angle, g_joints_init[i].max_angle, &angle);
		if (ret != 0) {
			LOG("robot angle map failed, joint_id:%d current_angle:%.2f\n", i, g_robot.joints[i].current_angle);
			return;
		}

		if (angle > g_joints_init[i].current_angle) { // 逆时针旋转
			dir = DIR_NEGATIVE;
		}

		if (g_joints_init[i].min_angle == 0 && g_joints_init[i].max_angle == 360) {
			if (fabs(angle - g_joints_init[i].current_angle) > 180) { // 寻找最小角度
				dir = -dir;
			}
		}
		
		LOG_FROM_ISR("[%d] current:%.2f, target:%.2f, dir:%d\n\n", i, angle, g_joints_init[i].current_angle, dir);
		g_robot.joints[i].current_angle = angle;

		robot_joint_rotate_to(i, dir, g_joints_init[i].current_angle, ROBOT_RESET_DEFAULT_VELOCITY, ROBOT_RESET_DEFAULT_ACCELERATION, true);
		vTaskDelay(100);
		g_robot.joints[i].current_angle = g_joints_init[i].current_angle;
	}

	g_robot.cur_pos.x = 0;
	g_robot.cur_pos.y = 0;
	g_robot.cur_pos.z = 0;
	robot_mqtt_joints_sync();
}

static struct position *robot_time_func_path_interpolation(uint32_t time_limit_ms, int *size)
{
    int path_size = 0;
	struct position pos = {0};
	int ret = 0;

    if (g_robot_time_func == NULL) {
        LOG("robot time func is null\n");
		return NULL;	
    }

	path_size = (time_limit_ms / ROBOT_INTERPOLATION_TIME_RESOLUTION); // 计算路径点数量
    struct position *path = (struct position*)malloc(sizeof(struct position) * path_size);
    if (path == NULL) {
        return NULL;
    }

    for (int i = 0; i < path_size; i++) {
        ret = g_robot_time_func(i * ROBOT_INTERPOLATION_TIME_RESOLUTION, &pos);
		if (ret != 0) {
			LOG("robot time func failed\n");
			free(path);
			return NULL;
		}

        path[i].x = pos.x;
        path[i].y = pos.y;
        path[i].z = pos.z;
    }

    *size = path_size;
    return path;
}

static void robot_time_func_move(uint32_t time_limit_ms)
{
	int ret;
	int path_size = 0;

	// 路径插值
	struct position *path = robot_time_func_path_interpolation(time_limit_ms, &path_size);
    if (path == NULL) {
        LOG("robot time func failed\n");
        return;
    }

	// 计算各路径点下的关节角度
    float *result = (float*)malloc(sizeof(float) * ROBOT_MAX_JOINT_NUM * path_size);
    if (result == NULL) {
        LOG("robot malloc failed\n");
        free(path);
        return;	
    }

	// 更新当前关节角度到算法
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        robot_kinematics_joint_angle_update_by_id(i, g_robot.joints[i].current_angle);
    }

	for (int i = 0; i < path_size; i++) {
        robot_kinematics_cal_T(T_0_6_reset, g_robot.T, &path[i]);
        ret = robot_kinematics_inverse((float *)g_robot.T, &result[i * ROBOT_MAX_JOINT_NUM], false);
        if (ret != 0) {
            LOG("robot kinematics inverse failed\n");
            free(path);
            free(result);
            return;
        }
		// 把本次计算的关节角度更新到算法，用于下次关节最优解计算
		robot_kinematics_joint_angle_update(&result[i * ROBOT_MAX_JOINT_NUM]);
		LOG("[%d] <%.2f %.2f %.2f> ", i, path[i].x, path[i].y, path[i].z);
		LOG("result: ");
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			LOG("%.2f ", result[i * ROBOT_MAX_JOINT_NUM + j]);
		}
		LOG("\n");
    }

	ret = robot_pid_run(path, path_size, result);
	if (ret == 0) {
		g_robot.cur_pos.x = path[path_size -1].x;
		g_robot.cur_pos.y = path[path_size -1].y;
		g_robot.cur_pos.z = path[path_size -1].z;
	}
	free(path);
	free(result);
}

static void robot_auto_move_interpolation(struct robot_event *event)
{
    int ret;
	// 插值计算各路径点
	int path_size = 0;
	struct position *target_pos = (struct position*)event->param;
    struct position *path = robot_path_interpolation_linear(target_pos, &path_size);
    if (path == NULL) {
        LOG("robot path interpolation failed\n");
        return;	
    }

    // 计算各路径点下的关节角度
    float *result = (float*)malloc(sizeof(float) * ROBOT_MAX_JOINT_NUM * path_size);
    if (result == NULL) {
        LOG("robot malloc failed\n");
        free(path);
        return;	
    }

	// 更新当前关节角度到算法
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        robot_kinematics_joint_angle_update_by_id(i, g_robot.joints[i].current_angle);
    }

    for (int i = 0; i < path_size; i++) {
        robot_kinematics_cal_T(T_0_6_reset, g_robot.T, &path[i]);
        ret = robot_kinematics_inverse((float *)g_robot.T, &result[i * ROBOT_MAX_JOINT_NUM], false);
        if (ret != 0) {
            LOG("robot kinematics inverse failed\n");
            free(path);
            free(result);
            return;
        }

		// 把本次计算的关节角度更新到算法，用于下次关节最优解计算
		robot_kinematics_joint_angle_update(&result[i * ROBOT_MAX_JOINT_NUM]);
		LOG("[%d] <%.2f %.2f %.2f> ", i, path[i].x, path[i].y, path[i].z);
		LOG("result: ");
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			LOG("%.2f ", result[i * ROBOT_MAX_JOINT_NUM + j]);
		}
		LOG("\n");
    }

	ret = robot_pid_run(path, path_size, result);
	if (ret == 0) {
		g_robot.cur_pos.x = target_pos->x;
		g_robot.cur_pos.y = target_pos->y;
		g_robot.cur_pos.z = target_pos->z;
	}
	free(path);
	free(result);
}

static inline bool robot_joint_is_reach(struct joint *joint, float target_angle)
{
	float current_angle = joint->current_angle;
	if ((joint->velocity == 0) && (fabs(current_angle - target_angle) <= ROBOT_JOINT_ANGLE_ERROR_RANGE)) {	// 速度为0，当前角度接近目标角度，已经到达
		return true;
	}

	if ((joint->velocity < 0) && (current_angle <= (target_angle + ROBOT_JOINT_ANGLE_ERROR_RANGE))) { // 速度为负，当前角度小于目标角度，已经到达
		return true;
	}

	if ((joint->velocity > 0) && (current_angle >= (target_angle - ROBOT_JOINT_ANGLE_ERROR_RANGE))) { // 速度为正，当前角度大于目标角度，已经到达
		return true;
	}

	return false;
}
static float robot_angle_normalize(float angle)
{
	// 默认入参角度在-360-720度范围内
	if (angle >= 360) {
		return angle - 360;	
	}

	if (angle < 0) {
		return angle + 360;	
	}
	return angle;
}

/**
 * @brief 计算当前角度与目标角度之间的最小差值，考虑角度循环。
 * 
 * 该函数会计算目标角度与当前角度的差值，并将差值映射到 -180 度到 180 度的范围内，
 * 以此确保得到的是两个角度之间的最小差值，考虑了角度在 0 - 360 度范围内循环的特性。
 * 
 * @param cur_angle 当前角度，单位为度。
 * @param target_angle 目标角度，单位为度。
 * @return float 当前角度与目标角度之间的最小差值，单位为度，范围在 -180 度到 180 度之间。
 */
static float robot_angle_diff(float cur_angle, float target_angle)
{
	float diff = target_angle - cur_angle;
	if (diff > 180) {
		diff -= 360;
	} else if (diff < -180) {
		diff += 360;
	}
	return diff;
}

float g_target_angle[ROBOT_MAX_JOINT_NUM] = {0};
float g_current_angle[ROBOT_MAX_JOINT_NUM] = {0};

static int robot_pid_run(struct position *path, int path_size, float *result)
{
	int p;
	int start_time = 0;
	int node_end_time = 0;
	float target_angle[ROBOT_MAX_JOINT_NUM] = {0};
	float pre_diff[ROBOT_MAX_JOINT_NUM] = {0};
	float intg_diff[ROBOT_MAX_JOINT_NUM] = {0};
	float total_error[ROBOT_MAX_JOINT_NUM] = {0};

	start_time = HAL_GetTick();
	for (p = 1; p < path_size; p++) {
		/* 急停中止：必须两个循环都查。外层管"还有没有下一个路径点"，
		   内层管"当前这个点还在持续发指令" —— 内层才是真正驱动电机的地方，
		   只查外层的话当前点会一直发到 node_end_time 才停。 */
		if (g_estop) {
			LOG("ESTOP: pid run aborted at path node %d/%d\n", p, path_size);
			return 1;
		}
		node_end_time = start_time + ROBOT_INTERPOLATION_TIME_RESOLUTION * p; // 本次path node结束时间, 为绝对时间
		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) { // 初始化
			target_angle[j] = robot_angle_normalize(result[p * ROBOT_MAX_JOINT_NUM + j]);
			g_target_angle[j] = target_angle[j]; // debug
		}

		while(HAL_GetTick() < node_end_time) { // 等待本次path node结束
			if (g_estop) {
				LOG("ESTOP: pid period aborted at path node %d/%d\n", p, path_size);
				return 1;
			}
			robot_pid_one_period(target_angle, intg_diff, pre_diff, total_error, 6);
		}
		robot_mqtt_joints_sync();
	}

	for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
		robot_joint_stop(j);
		vTaskDelay(ROBOT_CAN_DELAY);
	}
	for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
		LOG("[jpint %d] ave_error:%.2f\n", j + 1, total_error[j]/path_size);
	}
	LOG("\nrobot pid run finished!!\n");
	return 0;
}

static void robot_joints_sync_to(struct robot_event *event)
{
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		robot_joint_rotate_to(i, DIR_POSITIVE, event->param[i],
					ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
		g_robot.joints[i].current_angle = event->param[i]; // 更新当前角度
	}
}

/**
 * @brief 机械臂控制任务函数，负责从事件队列中接收事件并进行相应处理。
 * 
 * 该任务会持续从事件队列中获取机械臂相关事件，根据事件类型调用不同的处理函数，
 * 实现对机械臂关节运动、自动运动、复位等操作的控制。
 * 
 * @param arg 任务参数，在本函数中未使用。
 */
static void robot_control_task(void *arg)
{
	(void)arg;
    LOG("robot control task runing!!!\n");
    
	struct robot_event event = {0};
	// 从队列中取出事件并处理
    while(xQueueReceive(g_robot.event_queue, &event, portMAX_DELAY) == pdPASS) {
        /* 急停中：丢弃队列里所有待执行事件，不再驱动任何电机。
           必须在这里挡一道 —— robot_pid_run() 的中止检查只能中断"正在跑"的
           插值，挡不住"排队中"的事件；不丢的话急停解除后它们会立刻执行。 */
        if (g_estop) {
            LOG("ESTOP active, dropped event type %d joint %d\n", event.type, event.joint_id);
            continue;
        }
        switch (event.type) {
            case ROBOT_JOINT_REL_ROTATE:
				LOG("[joint_id: %d] ROBOT_JOINT_REL_ROTATE %f\n", event.joint_id, event.param[0]);
                robot_joint_rotate_to(event.joint_id, DIR_POSITIVE,event.param[0], 
						ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, false);
                break; 
			case ROBOT_JOINT_ABS_ROTATE:
				LOG("[joint_id: %d] ROBOT_JOINT_ABS_ROTATE %f\n", event.joint_id, event.param[0]);
				robot_joint_rotate_to(event.joint_id, DIR_POSITIVE, event.param[0], 
					ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
				break;
			case ROBOT_LIMIT_SWITCH_EVENT:
				LOG("[joint_id: %d] ROBOT_LIMIT_SWITCH_EVENT\n", event.joint_id);
				robot_joint_limit_post_handle(event.joint_id);
				break;
			case ROBOT_AUTO_EVENT:
				LOG("ROBOT_AUTO_EVENT\n");
				robot_auto_move_interpolation(&event);
				break;
			case ROBOT_TIMIE_FUNC_EVENT:
				LOG("ROBOT_TIMIE_FUNC_EVENT\n");
				robot_time_func_move((uint32_t)(event.param[0]));
				break;
			case ROBOT_HARD_RESET_EVENT:
				LOG("ROBOT_HARD_RESET_EVENT\n");
				robot_joint_hard_reset();
				break;
			case ROBOT_SOFT_RESET_EVENT:
				LOG("ROBOT_SOFT_RESET_EVENT\n");
				robot_joint_soft_reset();
				break;
			case ROBOT_TEST_EVENT:
				LOG("ROBOT_RESET_EVENT\n");
				robot_update_current_angle(event.joint_id);
				break;
			case ROBOT_REMOTE_CONTROL_EVENT:
				LOG("ROBOT_REMOTE_CONTROL_EVENT\n");
				robot_pid_remote();
				break;
			case ROBOT_JOINTS_SYNC_EVENT:
				LOG("ROBOT_JOINTS_SYNC_EVENT\n");
				robot_joints_sync_to(&event);
				break;
			default:
				LOG("robot event type error\n");
        }
    }
}

static void robot_pid_one_period(float *target_angle, float *intg_error, float *pre_error, float *total_error, int joint_num)
{
	float error = 0;
	float v;
	uint32_t pid_end_time = HAL_GetTick() + ROBOT_PID_PERIOD;
	for (int j = 0; j < joint_num; j++) {
		robot_update_current_angle(j);
		g_current_angle[j] = g_robot.joints[j].current_angle; // debug
		error = robot_angle_diff(g_robot.joints[j].current_angle, target_angle[j]);
		intg_error[j] += error;
		if (total_error != NULL) {
			total_error[j] += fabs(error);
		}

		v = ROBOT_PID_KP * error + ROBOT_PID_KI * intg_error[j] + ROBOT_PID_KD * (error - pre_error[j]);
		pre_error[j] = error;
		robot_joint_veloccity_to(j, v, ROBOT_JOINT_DEFAULT_ACCELERATION);
	}

	uint32_t time = HAL_GetTick();
	if (time < pid_end_time) {
		vTaskDelay(pid_end_time - time);
	}
}

struct position g_pos = {0};	// debug
static int robot_pid_remote(void)
{
	uint64_t end_time = 0;
	float target_angle[ROBOT_MAX_JOINT_NUM] = {0};
	float pre_error[ROBOT_MAX_JOINT_NUM] = {0};
	float intg_error[ROBOT_MAX_JOINT_NUM] = {0};
	struct position pos = {0};
	int ret;
	float T[4][4] = {0};
	int error_count = 0;

	LOG("wait robot reset....\n");
	vTaskDelay(3000);
	LOG("robot into remote mode!!!!\n");

	end_time = HAL_GetTick();
	while(ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		end_time += ROBOT_REMOTE_TIME_RESOLUTION; // 本次路径跟踪结束时间, 为绝对时间

		// 更新目标位置
		g_pos.x = g_robot.cur_pos.x + g_remote_control.vx * ROBOT_REMOTE_TIME_RESOLUTION / 1000;
		g_pos.y = g_robot.cur_pos.y + g_remote_control.vy * ROBOT_REMOTE_TIME_RESOLUTION / 1000;
		g_pos.z = g_robot.cur_pos.z + g_remote_control.vz * ROBOT_REMOTE_TIME_RESOLUTION / 1000;
		robot_kinematics_cal_T(T_0_6_reset, T, &g_pos);
		ret = robot_kinematics_inverse((float *)T, &g_remote_control.result[0], false);
		if (ret < 0) {
			error_count++;
			if (error_count >= 10) {
				LOG("robot kinematics inverse failed\n");
				error_count = 0;
			}
			
			for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
				robot_joint_stop(j);
				vTaskDelay(ROBOT_CAN_DELAY);
			}
			// vTaskDelay(ROBOT_INTERPOLATION_TIME_RESOLUTION);
			continue;
		}
		error_count = 0;
		robot_kinematics_joint_angle_update(&g_remote_control.result[0]);
		g_robot.cur_pos.x = g_pos.x;
		g_robot.cur_pos.y = g_pos.y;
		g_robot.cur_pos.z = g_pos.z;
		robot_joint_veloccity_to(4, g_remote_control.rx, ROBOT_JOINT_DEFAULT_ACCELERATION);
		robot_joint_veloccity_to(5, g_remote_control.ry, ROBOT_JOINT_DEFAULT_ACCELERATION);

		for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) { // 初始化
			target_angle[j] = robot_angle_normalize(g_remote_control.result[j]);
			g_target_angle[j] = target_angle[j]; // debug
		}

		while(HAL_GetTick() < end_time) { // 等待本次路径跟踪结束
			robot_pid_one_period(target_angle, intg_error, pre_error, NULL, 4);
		}
	}

	for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
		robot_joint_stop(j);
		vTaskDelay(ROBOT_CAN_DELAY);
	}

	LOG("\nrobot remote disable!!\n");
	return 0;
}

/**
 * @brief 初始化机械臂控制系统。
 * 
 * 此函数用于完成机械臂系统的初始化工作，包括初始化关节数据、初始位姿矩阵，
 * 创建事件队列以及启动机械臂控制任务。
 */
/* ---------------------------------------------------------------------------
 * 电机上电初始化：闭环位置模式 + 使能
 *
 * 为什么必须有：之前固件在 robot_joint_rotate_to() 里直接发 Emm_V5_Pos_Control，
 * 但从未设过控制模式、也从未使能。Emm_V5 的电机上电后默认不吃位置指令，
 * 线圈不上电就没有保持转矩——现象是"命令全都到位了（回显、队列、计数器全对），
 * 但机械臂一动不动"，而且从固件侧看不出任何异常。
 *
 * 两个步骤缺一不可：
 *   Emm_V5_Modify_Ctrl_Mode(addr, false, 2)  2 = 闭环模式（P_Pul 菜单）
 *   Emm_V5_En_Control(addr, true, false)      使能，线圈上电
 *
 * can_SendCmd() 只往 FDCAN TX FIFO 入队、不等应答，所以给没接电机的地址
 * （CAN 3~6）发指令完全无害，不会阻塞。
 *
 * 注意：这会让机械臂带电并产生保持转矩。如果当前构型下某个关节会因此
 * 受力位移，先用手托住或先点急停。 */
void robot_motor_enable_all(void)
{
    for (uint32_t i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        uint32_t addr = i + 1;              /* 各关节 CAN 地址从 1 开始 */
        Emm_V5_Modify_Ctrl_Mode((uint8_t)addr, false, 2);   /* 闭环模式 */
        HAL_Delay(2);
        Emm_V5_En_Control((uint8_t)addr, true, false);      /* 使能 */
        HAL_Delay(2);
    }
    LOG("motors: closed-loop position mode + enabled (CAN addr 1..%d)\n",
        ROBOT_MAX_JOINT_NUM);
}

void robot_init(void)
{
    /* 初始 */
    memcpy(g_robot.joints, g_joints_init, sizeof(g_joints_init));
    memcpy(g_robot.T, T_0_6_reset, sizeof(T_0_6_reset));

	g_robot.event_queue = xQueueCreate(ROBOT_MAX_EVENT_NUM, sizeof(struct robot_event));
    if (g_robot.event_queue == NULL) {
      	LOG("create robot event queue failed\n");
      	return; 
  	}

    g_robot.cmd_queue = xQueueCreate(ROBOT_CMD_MAX_NUM, sizeof(struct robot_cmd));
    if (g_robot.cmd_queue == NULL) {
    	LOG("create robot cmd queue failed\n");
    	return;
    }

	/* 创建robot控制任务(主程序) */
  	osThreadAttr_t task_attributes = { .name = "robot_control_task", 
                                     .stack_size = ROBOT_CONTROL_TASK_STACK_SIZE, 
                                     .priority = ROBOT_CONTROL_TASK_PRIORITY};
  	g_robot.control_handle = osThreadNew((osThreadFunc_t)robot_control_task, NULL, &task_attributes);
	if (g_robot.control_handle == NULL) {
		LOG("create robot control task failed\n");
		return;
	}

	/* 创建robot cmd 字符串处理任务 */
	task_attributes.name = "robot_cmd_service";
	task_attributes.stack_size = ROBOT_CMD_SERVICE_STACK_SIZE;
	task_attributes.priority = ROBOT_CMD_SERVICE_PRIORITY;
	g_robot.cmd_service_handle = osThreadNew((osThreadFunc_t)robot_cmd_service, NULL, &task_attributes);
	if (g_robot.cmd_service_handle == NULL) {
		LOG("create robot cmd service task failed\n");	
		return;
	}

	/* 电机上电：闭环位置模式 + 使能。放在任务创建之后、调度器启动之前，
	   CAN 外设此时已由 MX_FDCAN1_Init() 初始化完毕，可以安全发帧。 */
	robot_motor_enable_all();

	// /* 创建 robot remote 任务*/
	// task_attributes.name = "robot_remote_service";
	// task_attributes.stack_size = ROBOT_REMOTE_SERVICE_STACK_SIZE;
	// task_attributes.priority = ROBOT_REMOTE_SERVICE_PRIORITY;
	// g_robot.remote_service_handle = osThreadNew((osThreadFunc_t)robot_remote_service, NULL, &task_attributes);
	// if (g_robot.remote_service_handle == NULL) {
	// 	LOG("create robot remote service task failed\n");	
	// 	return;
	// }
}

int robot_send_joints_sync_event(float *angles)
{
	if (g_robot.event_queue == NULL) {
		return -1;	
	}

	struct robot_event event = {0};
	event.type = ROBOT_JOINTS_SYNC_EVENT;
	memcpy(event.param, angles, sizeof(float) * ROBOT_MAX_JOINT_NUM);
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_rel_rotate_event(uint8_t joint_id, float angle)
{
	struct robot_event event = {0};
	event.type = ROBOT_JOINT_REL_ROTATE;
	event.joint_id = joint_id;
	event.param[0] = angle;
    if (xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT) == pdPASS) {
        g_dbg_event_queued++;
    }
    return (int)pdPASS;
}

int robot_send_remote_event(void)
{
	struct robot_event event = {0};
	event.type = ROBOT_REMOTE_CONTROL_EVENT;
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_abs_rotate_event(uint8_t joint_id, float angle)
{
	struct robot_event event = {0};
	event.type = ROBOT_JOINT_ABS_ROTATE;
	event.joint_id = joint_id;
	event.param[0] = angle;
    return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

int robot_send_auto_event(struct position *pos)
{
	struct robot_event event = {0};
	event.type = ROBOT_AUTO_EVENT;
	memcpy(event.param, pos, sizeof(struct position));
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
};

int robot_send_time_func_event(float time_limit_ms)
{
	struct robot_event event = {0};
	event.type = ROBOT_TIMIE_FUNC_EVENT;
	event.param[0] = time_limit_ms;
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
};

int robot_send_reset_event(bool hard_reset)
{
	struct robot_event event = {0};
	if (hard_reset) {
		event.type = ROBOT_HARD_RESET_EVENT;	
	} else {
		event.type = ROBOT_SOFT_RESET_EVENT;
	}
	return (int)xQueueSendToBack(g_robot.event_queue, &event, ROBOT_CMD_QUEUE_TIMEOUT);
}

void robot_cmd_send_from_isr(char *cmd, enum cmd_type type)
{
	if (g_robot.cmd_queue == NULL) {
		return;	
	}

	struct robot_cmd robot_cmd = {0};
	robot_cmd.type = type;
	int len = strlen(cmd);
	strncpy(robot_cmd.cmd, cmd, (len >= ROBOT_CMD_LENGTH) ? ROBOT_CMD_LENGTH - 1: len);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendToBackFromISR(g_robot.cmd_queue, &robot_cmd, &xHigherPriorityTaskWoken) == pdPASS) {
        g_dbg_cmd_queued++;
    }
    /* 唤醒被命令唤醒的高优先级任务。少了这一句，robot_cmd_service 不会立刻跑，
       要等下一次 SysTick（1ms）才会被调度——功能上仍会执行，但响应慢一拍。 */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static int robot_joint_pin2id(uint16_t pin)
{
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        if (g_robot.joints[i].limit_gpio_pin == pin) {
            return i;
        }	
    }
    return -1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    int joint_id = robot_joint_pin2id(GPIO_Pin);
    if (joint_id < 0) {
        /* 不属于任何关节的 EXTI 线（如 PE0/PE1 误配、或未映射引脚），
         * 直接返回，避免越界访问 g_robot.joints[]。 */
        return;
    }
    robot_joint_limit_happend((uint8_t)joint_id);
    LOG_FROM_ISR("joint limit switch happened, joint id: %d\n", joint_id);
    /* 注意：不要在这里调 __HAL_GPIO_EXTI_CLEAR_IT()。
     * HAL_GPIO_EXTI_IRQHandler() 已经清过 pending 标志再回调，
     * 再清一次会把两次清之间到来的边沿吞掉。 */
}

static int robot_mqtt_joints_sync(void)
{
#if defined(ROBOT_MQTT_ENABLE) && (ROBOT_MQTT_ENABLE == 1)
	char msg[256] = {0};
	snprintf(msg, sizeof(msg), "[PC][%d][%.2f %.2f %.2f %.2f %.2f %.2f]", ROBOT_JOINTS_SYNC_EVENT,
				g_robot.joints[0].current_angle, g_robot.joints[1].current_angle,
				g_robot.joints[2].current_angle, g_robot.joints[3].current_angle,
				g_robot.joints[4].current_angle, g_robot.joints[5].current_angle);
	return esp8266_publish_message(MQTT_TOPIC, msg, 0, 0);
#else
	return 0;
#endif
}

#if defined(ROBOT_MQTT_ENABLE) && (ROBOT_MQTT_ENABLE == 1)
//static void robot_mqtt_sync_task(void)
//{
//	vTaskDelay(1000); // 等待esp8266初始化完成
//
//	while (1) {
//		for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
//			robot_update_current_angle(i); // 更新当前角度
//		}
//
//		int ret = robot_mqtt_joints_sync();
//		if (ret == 0) {
//			LOG("robot mqtt sync failed, ret:%d\n", ret);
//		}
//		vTaskDelay(ROBOT_MQTT_SYNC_TIME);
//	}
//}

//static int robot_mqtt_sync_task_init(void)
//{
//	// 创建低优先级任务，定期往MQTT服务器同步关节角度
//  	osThreadAttr_t task_attributes = { .name = "robot_mqtt_sync_task",
//                                     .stack_size = ROBOT_MQTT_SYNC_TASK_STACK_SIZE,
//                                     .priority = ROBOT_MQTT_SYNC_TASK_PRIORITY};
//  	g_robot.mqtt_sync_task_handle = osThreadNew((osThreadFunc_t)robot_mqtt_sync_task, NULL, &task_attributes);
//	if (g_robot.mqtt_sync_task_handle == NULL) {
//		LOG("create robot mqtt sync task failed\n");
//		return -1;
//	}
//	return 0;
//}
#endif	// defined(ROBOT_MQTT_ENABLE) && (ROBOT_MQTT_ENABLE == 1)

void robot_cmd_service(void)
{
	struct robot_cmd rb_cmd = {0};

#if defined(ROBOT_MQTT_ENABLE) &&  (ROBOT_MQTT_ENABLE == 1)
	int ret;
	LOG("robot mqtt service init...    \n");
	HAL_Delay(1000);	// 等待esp8266初始化完成
    ret = esp8266_mqtt_init();
	if (!ret) {
		LOG("robot mqtt service init failed\n");
		return;	
	}

	// ret = robot_mqtt_sync_task_init();
	// if (ret != 0) {
	// 	LOG("robot mqtt sync task init failed\n");
	// 	return;	
	// }

	LOG("robot mqtt service init successed\n");
#endif

	while(xQueueReceive(g_robot.cmd_queue, &rb_cmd, portMAX_DELAY) == pdPASS) {
		switch (rb_cmd.type)
		{
			case CMD_TYPE_UART1:
				robot_uart1_handle(&rb_cmd);
				break;
			
			case CMD_TYPE_MQTT:
				robot_mqtt_handle(&rb_cmd);
				break;
			
			default:
				break;
		}
	}
}

/**
 * @brief 对机械臂运动路径进行线性插值。
 * 
 * 该函数根据目标位置和当前位置，计算出机械臂运动路径上的插值点。
 * 通过线性插值的方式，将从当前位置到目标位置的路径划分为多个点，
 * 以实现机械臂的平滑运动。
 * 
 * @param target 指向目标位置结构体的指针，包含目标位置的x、y、z坐标。
 * @param size 指向整数的指针，用于返回插值点的数量。
 * @return struct position* 指向包含插值点的位置结构体数组的指针，
 *         若内存分配失败则返回NULL。
 */
static struct position *robot_path_interpolation_linear(struct position *target, int *size)
{
    float dx = target->x - g_robot.cur_pos.x;
    float dy = target->y - g_robot.cur_pos.y;
    float dz = target->z - g_robot.cur_pos.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    int numPoints = (int)(ceil(distance)/ROBOT_INTERPOLATION_RESOLUTION) + 1; // 加1是因为包括起点和终点
    *size = numPoints;

    struct position* path = (struct position*)malloc(numPoints * sizeof(struct position));
    if (path == NULL) {
        return NULL;
    }

    // 计算每一步的增量
    double step_x = dx / (numPoints - 1);
    double step_y = dy / (numPoints - 1);
    double step_z = dz / (numPoints - 1);

    // 插值计算每个点的坐标
    for (int i = 0; i < numPoints; i++) {
        path[i].x = g_robot.cur_pos.x + i * step_x;
        path[i].y = g_robot.cur_pos.y + i * step_y;
        path[i].z = g_robot.cur_pos.z + i * step_z;
    }

    return path;
}

static int robot_update_current_angle(uint8_t joint_id)
{
	struct joint *joint = &g_robot.joints[joint_id];
	vTaskSuspendAll();
	can.rxFrameFlag = false;
	uint32_t start_tick = HAL_GetTick();
	while(!can.rxFrameFlag) {
		if ((HAL_GetTick() - start_tick) > ROBOT_CAN_TIMEOUT) {
			LOG("joint %u update current angle timeout.\n", joint_id);
			xTaskResumeAll();
			return 1;
		}
		Emm_V5_Read_Sys_Params(joint_id + 1, S_CPOS);
		HAL_Delay(1);
	}
	
	taskENTER_CRITICAL();
	xTaskResumeAll();
	uint8_t id = (uint8_t)(can.CAN_RxMsg.Identifier >> 8) - 1;
	if ((can.rxData[0] != 0x36) || (can.rxData[6] != 0x6b) || (id != joint_id)) { // 读取失败
		taskEXIT_CRITICAL();
		return 1;
	}

	float angle = 0;
	for (int i = 5; i >= 2; i--) {
		angle += (float)(((uint32_t)can.rxData[i]) << ((5 - i) << 3));
	}

	if (can.rxData[1] == 0x01) { // 负数
		angle = -angle;
	}

	// 修正为关节方向
	if (joint->postive_direction == MOTOR_DIR_CCW) { // 修正为正方向
		angle = -angle;
	}
	
	taskEXIT_CRITICAL();
	// 转换为弧度
	angle = angle * 360 / 65536 / joint->reduction_ratio + g_joints_init[joint_id].current_angle;
	joint->current_angle = robot_angle_normalize(angle);
	return 0;
}

static void robot_joint_stop(uint8_t joint_id)
{
	vTaskSuspendAll();
	can.rxFrameFlag = false;
	uint32_t start_tick = HAL_GetTick();
	while(!can.rxFrameFlag) {
		if ((HAL_GetTick() - start_tick) > ROBOT_CAN_TIMEOUT) {
			LOG("joint %u stop timeout.\n", joint_id);
			xTaskResumeAll();
			return 1;
		}
		Emm_V5_Stop_Now(joint_id + 1, false);
		HAL_Delay(1);
	}
	xTaskResumeAll();
	g_robot.joints[joint_id].velocity = 0;
	return 0;
}

static void robot_joint_stop_from_isr(uint8_t joint_id)
{
	Emm_V5_Stop_Now(joint_id + 1, false);
	g_robot.joints[joint_id].velocity = 0;
}

static int time_func_circle(uint32_t time_ms, struct position *pos)
{
	float angle_vel = 2 * M_PI / 10; // 角速度, 10s一个周期
	float r = 30;
	float first_x = 10;
	
	if (time_ms < 1000) {	// 前1S移动10mm
		pos->x = 0;
		pos->z = 0;
		pos->y = (-first_x) * time_ms / 1000;
		return 0;
	}
	
	pos->z = 0;	// 固定高度
	time_ms -= 1000;
	// 半径为50mm
	pos->x = r * sin(angle_vel * time_ms / 1000);
	pos->y = (r * cos(angle_vel * time_ms / 1000) - r - first_x);
	return 0;
}
