//
// Created by 22560 on 25-4-5.
//

#ifndef ARM_H
#define ARM_H

#include "main.h"

#define HOMING_POSTURE (0) //init状态，默认使能后等待pitch3归位的状态
#define BASE_POSTURE (1) //base状态，所有零点初始化结束后的状态

extern double Arm_Posture[][4];

void Arm_Motor_Enable();
void Arm_Motor_Disable();
void Arm_Init();
void Arm_Motor_Pos_cmd(uint8_t Posture);

#endif //ARM_H
