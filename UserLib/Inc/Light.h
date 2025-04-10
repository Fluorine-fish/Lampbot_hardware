//
// Created by 22560 on 25-4-10.
//

#ifndef LIGHT_H
#define LIGHT_H

#include "main.h"
#include "can.h"

typedef struct {
    uint8_t lights[2];

    CAN_HandleTypeDef* hcan;
} Light_TypeDef;

void Light_Init(Light_TypeDef* light, CAN_HandleTypeDef* hcan);

/**
 * @brief 计算指定色温指定亮度的对应channel值
 * @param Temperature 输入色温， 3000-6500
 * @param Light 输入亮度 0-70
 */
void Light_Ctrl(Light_TypeDef* light, uint16_t Temperature, uint8_t Light);


#endif //LIGHT_H
