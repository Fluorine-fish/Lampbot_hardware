//
// Created by 22560 on 25-3-23.
//

#ifndef PID_H
#define PID_H

#define ANGLE_PID_MAX_SPEED (1500)
#define ANGLE_PID_MIN_SPEED (-1500)

#include "main.h"

/**
 * @brief
 *
 */
typedef struct PID_Param {
    float kp,ki,kd;
    int16_t limit,target,error,measure,error_sum,last_error;
    int32_t p_out,i_out,d_out,out,last_out;
}PID_Param;

void PID_Solution(PID_Param *param,int16_t measure,int16_t target);
void PID_Angle(PID_Param *param,int16_t measure,int16_t target);
void Angle_Calc(int16_t raw_angle);

#endif //PID_H
