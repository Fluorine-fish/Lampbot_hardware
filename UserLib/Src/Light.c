//
// Created by 22560 on 25-4-10.
//

#include "Light.h"

void Light_Init(Light_TypeDef* light, CAN_HandleTypeDef* hcan) {
    light->hcan = hcan;
}

void Light_Ctrl(Light_TypeDef* light, const uint16_t Temperature, const uint8_t Light) {
    const uint8_t channel1 = (Temperature - 3000) / 3500.0 * Light;
    const uint8_t channel2 = (6500 - Temperature) / 3500.0 * Light;
    light->lights[0] = (channel1 > 148 ? 148 : channel1) < 0 ? 0 : channel1 > 148 ? 148 : channel1;
    light->lights[1] = (channel2 > 148 ? 148 : channel2) < 0 ? 0 : channel2 > 148 ? 148 : channel2;

    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x150;
    TxHeader.IDE = CAN_ID_STD; //使用标准帧格式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧类型
    TxHeader.DLC = 0x08;

    HAL_CAN_AddTxMessage(light->hcan, &TxHeader, light->lights, (uint32_t*)CAN_TX_MAILBOX0);
}
