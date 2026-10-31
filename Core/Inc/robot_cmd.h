#ifndef __ROBOT_CMD_H__
#define __ROBOT_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "robot.h"
#include <stdint.h>

struct robot_cmd_info
{
    char *event_type;
    int (*cmd_func)(float *args);
    /* 该命令至少需要多少个数值参数。
     *
     * 为什么必须有：newlib-nano 的 sscanf 默认不支持 %f（要靠链接期
     * -u _scanf_float 拉进来）。一旦漏链接，sscanf 会**静默**返回 1、
     * 所有参数保持 0 —— 于是 "rel_rotate 0 5" 被当成 "rel_rotate 0 0"，
     * 算出 steps=0，电机收到"移动 0 个脉冲"，回"成功"，然后一动不动。
     * 这个坑曾经把整条调试链带偏（误判成 CAN 帧格式/波特率/驱动菜单）。
     * 有 min_args 之后，参数没解析出来会当场报错，不会再静默通过。 */
    uint8_t min_args;
};

extern void robot_mqtt_handle(struct robot_cmd *rb_cmd);
extern void robot_uart1_handle(struct robot_cmd *rb_cmd);

/* 开机自检：验证 sscanf 的 %f 真的可用。结果写 g_dbg_scanf_float_ok /
 * g_dbg_selftest_angle，可纯用 J-Link 读取，不依赖 USB 通道。 */
extern void robot_cmd_selftest(void);
extern volatile uint32_t g_dbg_scanf_float_ok;
extern volatile float    g_dbg_selftest_angle;

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_CMD_H__ */
