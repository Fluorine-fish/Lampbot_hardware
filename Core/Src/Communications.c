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

motor_t motor_6020 = {0, 0, 0, 0};
motor_t M3508_1 = {0, 0, 0, 0};
motor_t M3508_2 = {0, 0, 0, 0};
motor_t M3508_3 = {0, 0, 0, 0};
motor_t M3508_4 = {0, 0, 0, 0};

DM_motor_t J4310_1 = {0, 0, 0, 0, 0, 0, 0, 0};
DM_motor_t J4310_2 = {0, 0, 0, 0, 0, 0, 0, 0};
DM_motor_t J4310_3 = {0, 0, 0, 0, 0, 0, 0, 0};
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

/**
 * @brief 达妙电机速度位置模式报文
 *
 * @param stdid
 * @param vel fp16类型的速度值
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_SpeedPosition_cmd(CAN_HandleTypeDef* hacn, uint32_t stdid, float vel, float pos) {
    motor_tx_message.StdId = stdid;
    motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
    motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
    motor_tx_message.DLC = 0x08;

    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;
    motor_can_send_data[0] = *pbuf;
    motor_can_send_data[1] = *(pbuf + 1);
    motor_can_send_data[2] = *(pbuf + 2);
    motor_can_send_data[3] = *(pbuf + 3);
    motor_can_send_data[4] = *vbuf;
    motor_can_send_data[5] = *(vbuf + 1);
    motor_can_send_data[6] = *(vbuf + 2);
    motor_can_send_data[7] = *(vbuf + 3);

    return HAL_CAN_AddTxMessage(hacn, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0);
}

//达妙相关函数；
/**
 * @brief 达妙电机使能命令
 *
 * @param stdid
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_Enable(
    uint32_t stdid) {
    motor_tx_message.StdId = stdid;
    motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
    motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = 0xFF;
    motor_can_send_data[1] = 0xFF;
    motor_can_send_data[2] = 0xFF;
    motor_can_send_data[3] = 0xFF;
    motor_can_send_data[4] = 0xFF;
    motor_can_send_data[5] = 0xFF;
    motor_can_send_data[6] = 0xFF;
    motor_can_send_data[7] = 0xFC;

    return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0);
}

/**
 * @brief 达妙电机失能命令
 *
 * @param stdid
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_Disable(
    uint32_t stdid) {
    motor_tx_message.StdId = stdid;
    motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
    motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = 0xFF;
    motor_can_send_data[1] = 0xFF;
    motor_can_send_data[2] = 0xFF;
    motor_can_send_data[3] = 0xFF;
    motor_can_send_data[4] = 0xFF;
    motor_can_send_data[5] = 0xFF;
    motor_can_send_data[6] = 0xFF;
    motor_can_send_data[7] = 0xFD;

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

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

void decode_motor_measure_DM(DM_motor_t* motor, uint8_t* data) {
    motor->p_int = (data[1] << 8) | data[2];
    motor->v_int = (data[3] << 4) | (data[4] >> 4);
    motor->t_int = ((data[4] & 0xF) << 8) | data[5];
    motor->position = uint_to_float(motor->p_int, -12.5, 12.5, 16); // (-12.5,12.5)
    motor->velocity = uint_to_float(motor->v_int, -45, 45, 12); // (-45.0,45.0)
    motor->torque = uint_to_float(motor->t_int, -18, 18, 12); // (-18.0,18.0)
    motor->CAN_ID = (data[0] & 0x0F);
    motor->Err = (data[0] & 0xF0) >> 4;
}

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
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

int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
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
        decode_motor_measure_DM(&J4310_1, rx_data);
    }
    if (rx_header.StdId == 0x012) {
        decode_motor_measure_DM(&J4310_2, rx_data);
    }
    if (rx_header.StdId == 0x013) {
        decode_motor_measure_DM(&J4310_3, rx_data);
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

HAL_StatusTypeDef cmd_Light(
    uint32_t stdid, uint8_t Channel1, uint8_t Channel2) {
    motor_tx_message.StdId = stdid;
    motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
    motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
    motor_tx_message.DLC = 0x08;

    motor_can_send_data[0] = Channel1;
    motor_can_send_data[1] = Channel2;
    motor_can_send_data[2] = 0xAA;
    motor_can_send_data[3] = 0xAA;
    motor_can_send_data[4] = 0xAA;
    motor_can_send_data[5] = 0xAA;
    motor_can_send_data[6] = 0xAA;
    motor_can_send_data[7] = 0xAA;

    return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, (uint32_t*)CAN_TX_MAILBOX0);
}

void Light_cmd() {
    cmd_Light(0x150, Light_Channel[0], Light_Channel[1]);
    HAL_Delay(50);
}
