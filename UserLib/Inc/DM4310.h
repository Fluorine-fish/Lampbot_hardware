//
// Created by 22560 on 25-4-10.
//

#ifndef DM4310_H
#define DM4310_H

#include "main.h"
#include "stdbool.h"

typedef enum {
    DM_CtrlMode_MIT = 0,
    DM_CtrlMode_SpeedPosition = 1,
    DM_CtrlMode_Speed = 2,
} DM_CtrlMode;

typedef struct {
    uint8_t id;
    DM_CtrlMode ctrl_mode;
    bool enabled;

    float position, velocity, torque; // 电机实时数据
    uint8_t Err;

    CAN_HandleTypeDef* hcan;
} DM4310_HandleTypeDef;

void DM4310_Init(DM4310_HandleTypeDef* dm4310, CAN_HandleTypeDef* hcan, uint8_t id, DM_CtrlMode ctrl_mode);

void DM_Enable(const DM4310_HandleTypeDef* dm4310);

void DM_Disable(const DM4310_HandleTypeDef* dm4310);

void DM4310_Ctrl(DM4310_HandleTypeDef* dm4310, float position, float speed);

void DM4310_Update(DM4310_HandleTypeDef* dm4310, uint8_t* data);

#endif //DM4310_H
