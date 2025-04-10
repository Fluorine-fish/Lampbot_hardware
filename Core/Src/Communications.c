//
// Created by 22560 on 25-3-18.
//

#include "stm32f407xx.h"
#include "can.h"
#include "main.h"
#include "Arm.h"
#include "PID.h"
#include "tim.h"
#include "Communications.h"
#include "DM4310.h"

motor_t motor_6020 = {0, 0, 0, 0};
motor_t M3508_1 = {0, 0, 0, 0};
motor_t M3508_2 = {0, 0, 0, 0};
motor_t M3508_3 = {0, 0, 0, 0};
motor_t M3508_4 = {0, 0, 0, 0};

DM4310_HandleTypeDef DM4310_1;
DM4310_HandleTypeDef DM4310_2;
DM4310_HandleTypeDef DM4310_3;
M2006_motor_t M2006_1 = {0, 0, 0};

motor_t H6215_1 = {0, 0, 0, 0};
motor_t H6215_2 = {0, 0, 0, 0};
uint8_t motor_can_send_data[8];
CAN_TxHeaderTypeDef motor_tx_message;

extern int32_t angle;
extern PID_Param PID_Speed_M2006_1;
extern PID_Param PID_Angle_M2006_1;
extern int32_t last_angle;
extern uint8_t Enable_flag;
extern uint8_t Light_Channel[2];

/**
 * @brief CAN过滤器初始化
 *
 */
void can_filter_init(void) {
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief 大疆一拖四电机报文
 *
 * @param stdid 帧头
 * @param motor1
 * @param motor2
 * @param motor3
 * @param motor4
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef cmd_motor(
    uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    motor_tx_message.StdId = stdid;
    motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
    motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = motor1 >> 8;
    motor_can_send_data[1] = motor1;
    motor_can_send_data[2] = motor2 >> 8;
    motor_can_send_data[3] = motor2;
    motor_can_send_data[4] = motor3 >> 8;
    motor_can_send_data[5] = motor3;
    motor_can_send_data[6] = motor4 >> 8;
    motor_can_send_data[7] = motor4;

    return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0);
}

/*解码函数*/
void decode_motor_measure(motor_t* motor, uint8_t* data) {
    motor->angle_ecd = (data[0] << 8) | data[1];
    motor->speed_rpm = (data[2] << 8) | data[3];
    motor->current_raw = (data[4] << 8) | data[5];
    motor->temperate = data[6];
}

void decode_motor_measure_M2006(M2006_motor_t* motor, uint8_t* data) {
    motor->angle_ecd = (data[0] << 8) | data[1];
    motor->raw_speed_rpm = (data[2] << 8) | data[3];
    motor->torque = (data[4] << 8) | data[5];
    motor->speed_rpm = motor->raw_speed_rpm / 36.0f;
}

/**
 * @brief can接收回调函数 用于解码
 *
 * @param hcan
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    //标识符=0x200+ID
    if (rx_header.StdId == 0x201) {
        decode_motor_measure_M2006(&M2006_1, rx_data);
    }
    //达妙在速度位置模式下接收为MasterID
    if (rx_header.StdId == 0x011) {
        DM4310_Update(&DM4310_1, rx_data);
    }
    if (rx_header.StdId == 0x012) {
        DM4310_Update(&DM4310_2, rx_data);
    }
    if (rx_header.StdId == 0x013) {
        DM4310_Update(&DM4310_3, rx_data);
    }
}

/** 以下是用于机械臂电机使用的计算函数  **/

/**
 *  输入弧度制角度
 */
void M2006_Angel(double target_angle, int16_t Max_speed) {
    target_angle = (target_angle > 0) ? ((target_angle < 2.2340214) ? target_angle : 2.2340214) : 0;
    int16_t angle2M = 2900.0 / 2.3 * target_angle + 230;
    angle2M = (angle2M > 250) ? ((angle2M < 2900) ? angle2M : 2900) : 250;

    PID_Angle_M2006_1.target = angle2M;

    Angle_Calc(M2006_1.angle_ecd);
    PID_Angle(&PID_Angle_M2006_1, angle / 36, PID_Angle_M2006_1.target, Max_speed);
    PID_Solution(&PID_Speed_M2006_1, M2006_1.raw_speed_rpm, PID_Angle_M2006_1.out);
    cmd_motor(0x200, PID_Speed_M2006_1.out, 0, 0, 0);
}
