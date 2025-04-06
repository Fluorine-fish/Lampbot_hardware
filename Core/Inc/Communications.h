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
    int p_int,v_int,t_int;						//这里可根据电机数目自行修改，读取三个电机的位置、速度、转矩
    float position,velocity,torque;
    int16_t CAN_ID;
    uint8_t Err;
}DM_motor_t;

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
HAL_StatusTypeDef cmd_motor(
  uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
HAL_StatusTypeDef DM_SpeedPosition_cmd(CAN_HandleTypeDef *hacn,uint32_t stdid, float vel, float pos);
HAL_StatusTypeDef DM_Enable(uint32_t stdid);
void decode_motor_measure_DM(DM_motor_t * motor, uint8_t * data);
void decode_motor_measure(motor_t * motor, uint8_t * data);
HAL_StatusTypeDef DM_Enable(uint32_t stdid);
HAL_StatusTypeDef DM_Disable(uint32_t stdid);
HAL_StatusTypeDef DM_H6215_MIT_cmd(uint32_t stdid, float Torque);
void M2006_Angel(double target_angle);
HAL_StatusTypeDef cmd_Light(
  uint32_t stdid, uint8_t Channel1,uint8_t Channel2);


#endif //COMMUNICATIONS_H
