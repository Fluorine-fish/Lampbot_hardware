//
// Created by 22560 on 25-3-23.
//

#include "PID.h"
#include "main.h"

PID_Param PID_Speed_M2006_1 = {12,0.2,1,6000,600,0,0,0,0,0,0,0,0,0};
PID_Param PID_Angle_M2006_1 = {8,0,0,1000,100,0,0,0,0,0,0,0,0,0};
int32_t angle=0;
int32_t last_angle=0;


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
    if(param -> error < 50 && param -> error > -50) param->error_sum = 0;

    //输出限幅防止超出限制
    if(param -> out >=16000) param->out = 16000;
    if(param -> out <= -16000) param->out = -16000;
}

void PID_Angle(PID_Param *param,int16_t measure,int16_t target) {
    // 纠正半圈路径
    // if(measure-target>4096) target+=8191;
    // else if(measure-target<-4096) measure+=8191;

    param->target = target;
    param->measure = measure;

    //输入角度限制
    if(param->target >= 2450) param->target = 2450;
    else if(param->target <= 200) param->target = 200;

    param->error = param->target - param->measure;
    param->error_sum += param->error;
    if(param->error_sum > param->limit) param->error_sum = param->limit;//i项限幅

    param->p_out=param->kp * param->error;
    param->i_out = param->ki * param->error_sum;
    param->d_out = param->kd * (param->error - param->last_error);

    param->out = param->p_out + param->i_out + param->d_out;
    param->last_error = param->error;

    //i项清零
    if(param -> error < 5 && param -> error > -5) param->error_sum = 0;

    //最大速度限制
    if(param -> out >=ANGLE_PID_MAX_SPEED) param->out = ANGLE_PID_MAX_SPEED;
    if(param -> out <= ANGLE_PID_MIN_SPEED) param->out = ANGLE_PID_MIN_SPEED;
}

void Angle_Calc(int16_t raw_angle) {
    int16_t delta_angle = raw_angle - last_angle;
    if(raw_angle - last_angle > 6000) delta_angle = raw_angle-last_angle - 8191;
    else if(raw_angle - last_angle <-6000) delta_angle = raw_angle -last_angle + 8191;
    else delta_angle = raw_angle - last_angle;

    if(angle + delta_angle > 8191*36) angle = angle + delta_angle - 8191*36;
    else if(angle + delta_angle < 0) angle = angle + delta_angle + 8191*36;
    else angle = angle + delta_angle;

    last_angle = raw_angle;
}

