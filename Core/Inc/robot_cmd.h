#ifndef __ROBOT_CMD_H__
#define __ROBOT_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "robot.h"

struct robot_cmd_info
{
    char *event_type;
    int (*cmd_func)(float *args);
};

extern void robot_mqtt_handle(struct robot_cmd *rb_cmd);
extern void robot_uart1_handle(struct robot_cmd *rb_cmd);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_CMD_H__ */
