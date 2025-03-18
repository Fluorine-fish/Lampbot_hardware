//
// Created by 22560 on 25-3-18.
//

#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include "main.h"
#include "stm32f407xx.h"

typedef struct
{
    uint16_t angle_ecd;
    int16_t speed_rpm;
    int16_t current_raw;
    int16_t temperate;
}motor_t;

void can_filter_init(void);
HAL_StatusTypeDef cmd_motor(
  uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
HAL_StatusTypeDef DM_Speed_cmd(uint32_t stdid, float vel);
HAL_StatusTypeDef DM_Enable(uint32_t stdid);
void decode_motor_measure_DM(motor_t * motor, uint8_t * data);
void decode_motor_measure(motor_t * motor, uint8_t * data);
HAL_StatusTypeDef DM_Enable(uint32_t stdid);
HAL_StatusTypeDef DM_Disable(uint32_t stdid);
HAL_StatusTypeDef DM_H6215_MIT_cmd(uint32_t stdid, float Torque);

#endif //COMMUNICATIONS_H
