//
// Created by 22560 on 25-3-23.
//

#include "PID.h"
#include "main.h"

PID_Param PID_M2006_1 = {12,0.2,0,6000,600,0,0,0,0,0,0,0,0};

void PID_Solution(PID_Param *param,int16_t measure,int16_t target) {
    param->target = target;
    param->measure = measure;
    param->error = param->target - param->measure;
    param->error_sum += param->error;
    if(param->error_sum > param->limit) param->error_sum = param->limit;//i项限幅

    param->p_out=param->kp * param->error;
    param->i_out = param->ki * param->error_sum;
    param->d_out = param->kd * (param->error - param->last_error);

    param->out = param->p_out + param->i_out + param->d_out;
    param->last_error = param->error;

    //输出限幅防止超出限制
    if(param -> out >=16000) param->out = 16000;
    if(param -> out <= -16000) param->out = -16000;
}

void PID_Angle(PID_Param *param,int16_t measure,int16_t target,int16_t max_speed) {
    // 纠正半圈路径
    if(measure-target>4096) target+=8191;
    else if(measure-target<-4096) measure+=8191;

    param->target = target;
    param->measure = measure;
    param->error = param->target - param->measure;
    param->error_sum += param->error;
    if(param->error_sum > param->limit) param->error_sum = param->limit;//i项限幅

    param->p_out=param->kp * param->error;
    param->i_out = param->ki * param->error_sum;
    param->d_out = param->kd * (param->error - param->last_error);

    param->out = param->p_out + param->i_out + param->d_out;
    param->last_error = param->error;

    //最大速度限制
    if(param -> out >=max_speed) param->out = max_speed;
    if(param -> out <= -max_speed) param->out = -max_speed;
}
