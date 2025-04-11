//
// Created by 22560 on 25-4-11.
//

#ifndef M2006_H
#define M2006_H

#include "main.h"

typedef struct {
    uint8_t id;

    uint16_t angle_ecd;
    int16_t raw_speed_rpm;
    int16_t speed_rpm;
    int16_t torque;

    CAN_HandleTypeDef* hcan;
} M2006_HandleTypeDef;

void M2006_Init(M2006_HandleTypeDef* M2006, CAN_HandleTypeDef* hcan, uint8_t id);

void M2006_Update(M2006_HandleTypeDef* M2006, uint8_t* data);

void M2006_Crtl_Currency(M2006_HandleTypeDef* M2006_1, int16_t cur);

#endif //M2006_H
