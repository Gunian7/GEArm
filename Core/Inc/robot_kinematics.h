#ifndef __ROBOT_KINEMATICS_H__
#define __ROBOT_KINEMATICS_H__

#include "robot.h"

#ifdef __cplusplus
extern "C" {    
#endif

#define ROBOT_KINEMATICS_RESULT_NUM   (4U) /* 解算结果数量 */
#define T_ROW_COL                     (4U) /* DH参数矩阵行列数 */

struct robot_kinematics {
    float T[T_ROW_COL][T_ROW_COL];                                  /* 目标T矩阵 */
    float result[ROBOT_KINEMATICS_RESULT_NUM][ROBOT_MAX_JOINT_NUM]; /* 解算结果 */
    uint32_t result_invalid_mask;                                   /* 解算结果无效掩码 */
};

void robot_kinematics_cal_T(const float T_in[4][4], float T_out[4][4], struct position *pos);
void robot_kinematics_show_result(void);
void robot_kinematics_show_T(float T[4][4]);
int robot_kinematics_inverse(float *T_target, float *result, int show);
void robot_kinematics_joint_angle_update_by_id(uint32_t joint_id ,float angle);
void robot_kinematics_joint_angle_update(float *joint_angle);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_KINEMATICS_H__ */
