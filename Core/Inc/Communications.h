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

typedef struct
{
    uint16_t angle_ecd;
    int16_t raw_speed_rpm;
    int16_t speed_rpm;
    int16_t torque;
}M2006_motor_t;

#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值

void can_filter_init(void);
void decode_motor_measure(motor_t * motor, uint8_t * data);
HAL_StatusTypeDef DM_H6215_MIT_cmd(uint32_t stdid, float Torque);
void M2006_Angel(double target_angle,int16_t Max_speed);
HAL_StatusTypeDef cmd_Light(
  uint32_t stdid, uint8_t Channel1,uint8_t Channel2);
void Light_cmd();
void Arm_Qucik_Off();


#endif //COMMUNICATIONS_H
