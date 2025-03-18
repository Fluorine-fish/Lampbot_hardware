//
// Created by 22560 on 25-3-18.
//

#include "can.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Remote.h"
#include "main.h"
#include "Communications.h"

extern CAN_TxHeaderTypeDef motor_tx_message;
extern uint8_t motor_can_send_data[8];

extern RC_t RC;
int16_t speed=10;
float motor2_speed_rpm=0;

float LPF2(float xin);

void motor_task(void const * argument)
{
    can_filter_init();

    HAL_Delay(1000);
    HAL_Delay(50);
    HAL_Delay(50);
    while (1) {

        osDelay(1);
    }
}

