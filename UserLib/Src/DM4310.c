//
// Created by 22560 on 25-4-10.
//

#include "DM4310.h"

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DM4310_Init(DM4310_HandleTypeDef* dm4310, CAN_HandleTypeDef* hcan, uint8_t id, DM_CtrlMode ctrl_mode) {
    dm4310->id = id;
    dm4310->ctrl_mode = ctrl_mode;
    dm4310->hcan = hcan;
}

void DM_Enable(const DM4310_HandleTypeDef* dm4310) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxBuffer[8];
    TxHeader.IDE = CAN_ID_STD; //使用标准帧格式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧类型
    TxHeader.DLC = 0x08;
    if (dm4310->ctrl_mode == DM_CtrlMode_SpeedPosition) {
        TxHeader.StdId = 0x100 + dm4310->id;
    }
    else if (dm4310->ctrl_mode == DM_CtrlMode_Speed) {
        TxHeader.StdId = 0x200 + dm4310->id;
    }
    else if (dm4310->ctrl_mode == DM_CtrlMode_MIT) {
        TxHeader.StdId = 0x000 + dm4310->id;
    }

    TxBuffer[0] = 0xFF;
    TxBuffer[1] = 0xFF;
    TxBuffer[2] = 0xFF;
    TxBuffer[3] = 0xFF;
    TxBuffer[4] = 0xFF;
    TxBuffer[5] = 0xFF;
    TxBuffer[6] = 0xFF;
    TxBuffer[7] = 0xFC;

    HAL_CAN_AddTxMessage(dm4310->hcan, &TxHeader, TxBuffer, (uint32_t*)CAN_TX_MAILBOX0);
}

void DM_Disable(const DM4310_HandleTypeDef* dm4310) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxBuffer[8];
    TxHeader.IDE = CAN_ID_STD; //使用标准帧格式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧类型
    TxHeader.DLC = 0x08;
    if (dm4310->ctrl_mode == DM_CtrlMode_SpeedPosition) {
        TxHeader.StdId = 0x100 + dm4310->id;
    }
    else if (dm4310->ctrl_mode == DM_CtrlMode_Speed) {
        TxHeader.StdId = 0x200 + dm4310->id;
    }
    else if (dm4310->ctrl_mode == DM_CtrlMode_MIT) {
        TxHeader.StdId = 0x000 + dm4310->id;
    }

    TxBuffer[0] = 0xFF;
    TxBuffer[1] = 0xFF;
    TxBuffer[2] = 0xFF;
    TxBuffer[3] = 0xFF;
    TxBuffer[4] = 0xFF;
    TxBuffer[5] = 0xFF;
    TxBuffer[6] = 0xFF;
    TxBuffer[7] = 0xFD;

    HAL_CAN_AddTxMessage(dm4310->hcan, &TxHeader, TxBuffer, (uint32_t*)CAN_TX_MAILBOX0);
}


void DM4310_Ctrl(DM4310_HandleTypeDef* dm4310, float position, float speed) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxBuffer[8];

    TxHeader.IDE = CAN_ID_STD; //使用标准帧格式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧类型
    TxHeader.DLC = 0x08;
    if (dm4310->ctrl_mode == DM_CtrlMode_SpeedPosition) {
        TxHeader.StdId = 0x100 + dm4310->id;
    }
    else if (dm4310->ctrl_mode == DM_CtrlMode_Speed) {
        TxHeader.StdId = 0x200 + dm4310->id;
    }
    else if (dm4310->ctrl_mode == DM_CtrlMode_MIT) {
        TxHeader.StdId = 0x000 + dm4310->id;
    }

    *(float*)TxBuffer = position;
    *(float*)(TxBuffer + 4) = speed;

    HAL_CAN_AddTxMessage(dm4310->hcan, &TxHeader, TxBuffer, (uint32_t*)CAN_TX_MAILBOX0);
}

void DM4310_Update(DM4310_HandleTypeDef* dm4310, uint8_t* data) {
    if ((data[0] & 0x0F) != dm4310->id) return;
    const int p_int = (data[1] << 8) | data[2];
    const int v_int = (data[3] << 4) | (data[4] >> 4);
    const int t_int = ((data[4] & 0xF) << 8) | data[5];
    dm4310->position = uint_to_float(p_int, -12.5f, 12.5f, 16); // (-12.5,12.5)
    dm4310->velocity = uint_to_float(v_int, -45, 45, 12); // (-45.0,45.0)
    dm4310->torque = uint_to_float(t_int, -18, 18, 12); // (-18.0,18.0)
    dm4310->Err = (data[0] & 0xF0) >> 4;
    if (dm4310->Err == 1) dm4310->enabled = true;
    else if (dm4310->Err == 0) dm4310->enabled = false;
}
