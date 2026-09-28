#ifndef __ROBOT_H__
#define __ROBOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "stdint.h"
#include "main.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "string.h"
#include <math.h>
#include "stdbool.h"

#define ROBOT_MAX_JOINT_NUM                 6
#define ROBOT_MAX_EVENT_NUM                 20

#define ROBOT_MQTT_ENABLE                   0U /* 使能MQTT服务 */

#define ROBOT_CMD_MAX_NUM                   50
#define ROBOT_CMD_LENGTH                    128
#define ROBOT_CMD_QUEUE_TIMEOUT             100  /* 发送命令队列超时时间ms */

#define ROBOT_CONTROL_TASK_STACK_SIZE       4096
#define ROBOT_CONTROL_TASK_PRIORITY         (osPriorityRealtime3)

#define ROBOT_CMD_SERVICE_STACK_SIZE        2048
#define ROBOT_CMD_SERVICE_PRIORITY          (osPriorityRealtime2)

#define ROBOT_MQTT_SYNC_TASK_STACK_SIZE     2048
#define ROBOT_MQTT_SYNC_TASK_PRIORITY       (osPriorityRealtime)

// #define ROBOT_REMOTE_SERVICE_STACK_SIZE     2048
// #define ROBOT_REMOTE_SERVICE_PRIORITY       (osPriorityRealtime1)
// #define ROBOT_REMOTE_RESULT_NUM             3

#define ROBOT_REMOTE_MAX_VELOCITY           (20.0f)  /* 最大速度mm/s */
#define ROBOT_REMOTE_MAX_RPM                (5.0f)   /* 最大旋转角速度 rpm */
#define ROBOT_REMOTE_TIME_RESOLUTION        (50)    /* 时间插值分辨率ms */

#define ROBOT_JOINT_DEFAULT_VELOCITY        (10.0f) /* 关节默认速度 */
#define ROBOT_JOINT_DEFAULT_ACCELERATION    200 /* 关节默认加速度 */

#define ROBOT_INTERPOLATION_TIME_RESOLUTION (100) /* 时间插值分辨率ms */
#define ROBOT_INTERPOLATION_RESOLUTION      (1.0f) /* 路径插值分辨率mm */

#define ROBOT_RESET_DEFAULT_ANGLE           360     /* 复位默认角度 */
#define ROBOT_RESET_DEFAULT_VELOCITY        (10.0f) /* 复位默认速度rpm */
#define ROBOT_RESET_DEFAULT_ACCELERATION    100     /* 复位默认加速度 */

/* ROBOT STATUS FLAG */
#define ROBOT_STATUS_LIMIT_ENABLE           0U /* 限位开关使能 */
#define ROBOT_STATUS_LIMIT_HAPPENED         1U /* 限位开关触发 */
#define ROBOT_STATUS_READY                  2U /* 当前已完成运动 */
#define ROBOT_STATUS_RMODE_ENABLE           3U /* 远程控制使能 */
#define ROBOT_STATUS_MQTT_CONNECTED         4U /* MQTT连接状态 */

#define ROBOT_STATUS_IS(x, status)          (((x) & (1 << status)) != 0)           /* 状态位是否触发 */
#define ROBOT_STATUS_SET(x, status)         (x = ((x) | (1 << status)))            /* 设置状态位触发状态 */
#define ROBOT_STATUS_CLEAR(x, status)       (x = ((x) & ~(1 << status)))           /* 清除状态位触发状态 */

#define ROBOT_CAN_DELAY                     5 /* CAN发送延时ms */

#define ROBOT_PID_KP                        (10.0f) /* PID参数 */
#define ROBOT_PID_KI                        (0.002f) /* PID参数 */
#define ROBOT_PID_KD                        (0.0f) /* PID参数 */
#define ROBOT_PID_PERIOD                    (20)   /* PID时间周期ms, 建议不小于10ms */

#define ROBOT_CAN_TIMEOUT                   (10) /* CAN超时时间ms */

/* 误差范围 */
#define ROBOT_ERROR_RANGE                   (1e-4f)
#define ROBOT_JOINT_ANGLE_ERROR_RANGE       (1e-1f)

#define ROBOT_MQTT_SYNC_TIME                (100) /* 同步时间间隔ms */

enum motor_dir
{
    MOTOR_DIR_CW = 0,     /* 电机正转 */
    MOTOR_DIR_CCW = 1,    /* 电机反转 */
};

enum dir
{
    DIR_POSITIVE = 1,     /* 正方向 */
    DIR_NEGATIVE = -1,     /* 负方向 */
};

struct joint
{
    float current_angle;
    enum motor_dir postive_direction;   /* 关节旋转正方向对应的电机旋转方向 */
    float reduction_ratio;
    GPIO_TypeDef *limit_gpio_port;      /* 限位开关GPIO端口 */
    uint16_t limit_gpio_pin;            /* 限位开关GPIO引脚 */
    float min_angle;                    /* 关节最小角度 */
    float max_angle;                    /* 关节最大角度 */
    enum dir reset_dir;                 /* 复位方向 */
    
    /* 上面数据用于初始化，请不要修改数据位置 */
    volatile uint32_t status;           /* 关节状态 */
    float velocity;                     /* 关节速度 */
    float acceleration;                 /* 关节加速度 */
};

/* 机械臂末端位置 */
struct position
{
    float x;
    float y;
    float z;
};

/* 机械臂姿态 */
struct rotate
{
    float x;
    float y;
    float z;
};

struct robot
{
    osThreadId_t control_handle;
    osThreadId_t cmd_service_handle;
    osThreadId_t mqtt_sync_task_handle;
    osThreadId_t remote_service_handle;
    float T[4][4];
    struct joint joints[ROBOT_MAX_JOINT_NUM];
    uint32_t status;
    QueueHandle_t event_queue;
    QueueHandle_t cmd_queue;
    struct position cur_pos;    /* 当前末端坐标系下的位置 */
    struct rotate cur_rot;
};

struct robot_remote_control
{
    float vx;   // 末端x方向速度mm/s
    float vy;   // 末端y方向速度mm/s
    float vz;   // 末端z方向速度mm/s
    float rx;   // 末端绕x轴旋转角度速度rpm
    float ry;   // 末端绕y轴旋转角度速度rpm
    float result[ROBOT_MAX_JOINT_NUM];
    int lock;
};

enum robot_event_type
{
    ROBOT_JOINT_REL_ROTATE = 0,     /* 相对旋转事件：指定关节相对当前位置旋转一定角度 */
    ROBOT_JOINT_ABS_ROTATE,         /* 绝对旋转事件：指定关节旋转到指定角度 */
    ROBOT_LIMIT_SWITCH_EVENT,       /* 限位开关事件：指定关节到达限位开关时触发事件 */
    ROBOT_AUTO_EVENT,               /* 自动事件：指定机械臂末端位置及姿态，算法自动控制各关节电机运动 */
    ROBOT_TIMIE_FUNC_EVENT,         /* 时间函数事件: 机械臂末端根据时间函数P(t)运动 */
    ROBOT_HARD_RESET_EVENT,         /* 硬件复位事件：使限位开关将机械臂复位到初始位置 */
    ROBOT_SOFT_RESET_EVENT,         /* 软件复位事件：使机械臂复位到初始位置 */
    ROBOT_TEST_EVENT,
    ROBOT_REMOTE_CONTROL_EVENT,     /* 远程控制事件：根据远程控制指令控制机械臂运动 */
    ROBOT_JOINTS_SYNC_EVENT,        /* 关节同步事件：同步机械臂关节状态 */
};

enum
{
    ROBOT_JOINT_1 = 0,
    ROBOT_JOINT_2,
    ROBOT_JOINT_3,
    ROBOT_JOINT_4,
    ROBOT_JOINT_5,
    ROBOT_JOINT_6,
};

enum cmd_type
{
    CMD_TYPE_UART1 = 0,
    CMD_TYPE_MQTT,
};

struct robot_event
{
    enum robot_event_type type;
    uint8_t joint_id;
    float param[6];
};

struct robot_cmd
{
    enum cmd_type type;
    char cmd[ROBOT_CMD_LENGTH];
};

typedef int(*robot_time_func)(uint32_t time_ms, struct position *pos); /* 时间函数 */

extern struct robot g_robot;       /* robot实例 */
extern const float D_H[6][4];      /* DH参数 */
extern const float T_0_6_reset[4][4];
extern const float joint_weight[ROBOT_MAX_JOINT_NUM];
extern volatile struct robot_remote_control g_remote_control;

void robot_init(void);
void robot_cmd_service(void);
void robot_cmd_send_from_isr(char *cmd, enum cmd_type type);
int robot_send_rel_rotate_event(uint8_t joint_id, float angle);
int robot_send_auto_event(struct position *pos);
int robot_send_reset_event(bool hard_reset);
int robot_send_abs_rotate_event(uint8_t joint_id, float angle);
int robot_send_remote_event(void);
int robot_send_time_func_event(float time_limit_ms);
uint32_t robot_joint_veloccity_to(uint32_t joint_id, float velocity, uint8_t acceleration);
HAL_StatusTypeDef CAN_Send_Msg(uint32_t StdId, uint8_t *pData, uint8_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_H__ */
