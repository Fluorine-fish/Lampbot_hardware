//
// Created by 22560 on 25-4-11.
//

#include "M2006.h"
#include "can.h"

void M2006_Init(M2006_HandleTypeDef* M2006,CAN_HandleTypeDef* hcan,uint8_t id) {
    M2006->id = id;
    M2006->hcan = hcan;
}

void M2006_Update(M2006_HandleTypeDef* M2006, uint8_t* data) {
    M2006->angle_ecd = (data[0] << 8) | data[1];
    M2006->raw_speed_rpm = (data[2] << 8) | data[3];
    M2006->torque = (data[4] << 8) | data[5];
    M2006->speed_rpm = M2006->raw_speed_rpm / 36.0f;
}

void M2006_Crtl_Currency(M2006_HandleTypeDef* M2006, int16_t cur) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxBuffer[8];

    TxHeader.StdId = 0x200;
    TxHeader.IDE = CAN_ID_STD; //使用标准帧格式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧类型
    TxHeader.DLC = 0x08;

    TxBuffer[0] = cur >> 8;
    TxBuffer[1] = cur;
    TxBuffer[2] = 0x00;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x00;
    TxBuffer[5] = 0x00;
    TxBuffer[6] = 0x00;
    TxBuffer[7] = 0x00;

    HAL_CAN_AddTxMessage(M2006->hcan, &TxHeader, TxBuffer, (uint32_t*)CAN_TX_MAILBOX0);
}
