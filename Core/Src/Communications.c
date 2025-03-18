//
// Created by 22560 on 25-3-18.
//

#include "stm32f407xx.h"
#include "can.h"
#include "cmsis_os.h"
#include "main.h"

//大疆电机数据存储结构体
typedef struct
{
  uint16_t angle_ecd;
  int16_t speed_rpm;
  int16_t current_raw;
  int16_t temperate;
}motor_t;

motor_t motor_6020={0,0,0,0};
motor_t M3508_1={0,0,0,0};
motor_t M3508_2={0,0,0,0};
motor_t M3508_3={0,0,0,0};
motor_t M3508_4={0,0,0,0};
motor_t J4310={0,0,0,0};
motor_t H6215_1={0,0,0,0};
motor_t H6215_2={0,0,0,0};
uint8_t motor_can_send_data[8];
CAN_TxHeaderTypeDef motor_tx_message;

/**
 * @brief CAN过滤器初始化
 *
 */
void can_filter_init(void)
{
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
  uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
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

  return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

/**
 * @brief 达妙电机速度模式报文
 *
 * @param stdid
 * @param vel fp16类型的速度值
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_Speed_cmd(
  uint32_t stdid, float vel)
{
  uint32_t send_mail_box;
  motor_tx_message.StdId = stdid;
  motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
  motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
  motor_tx_message.DLC = 0x08;

  char* p=&vel;
  motor_can_send_data[0] = p[0];
  motor_can_send_data[1] = p[1];
  motor_can_send_data[2] = p[2];
  motor_can_send_data[3] = p[3];
  motor_can_send_data[4] = 0;
  motor_can_send_data[5] = 0;
  motor_can_send_data[6] = 0;
  motor_can_send_data[7] = 0;

  return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

//达妙H6215相关函数；
/**
 * @brief 达妙电机使能命令
 *
 * @param stdid
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_Enable(
  uint32_t stdid)
{
  uint32_t send_mail_box;
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

  return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

/**
 * @brief 达妙电机失能命令
 *
 * @param stdid
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_Disable(
  uint32_t stdid)
{
  uint32_t send_mail_box;
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

  return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

/**
 * @brief 达妙电机用浮点转换整形
 *
 * @param X_float
 * @param X_min
 * @param X_max
 * @param bits
 * @return int
 */
static int float_to_uint(float X_float, float X_min, float X_max,
  int bits){
    float span = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}

HAL_StatusTypeDef DM_H6215_MIT_cmd(uint32_t stdid, float Torque)
{
  uint32_t send_mail_box;
  motor_tx_message.StdId = stdid;
  motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
  motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
  motor_tx_message.DLC = 0x08;

  //根据上位机的电机转矩最大最小限位
  float T_Max = 17.0;
  float T_Min = -17.0;
  int KD_Tmp = float_to_uint(0,0,5,12);
  int Torque_Tmp = float_to_uint(Torque,-T_Max,T_Max,12);


  motor_can_send_data[0] = 0;
  motor_can_send_data[1] = 0;
  motor_can_send_data[2] = 0;
  motor_can_send_data[3] = 0;
  motor_can_send_data[4] = 0;
  motor_can_send_data[5] = 0;
  motor_can_send_data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (Torque_Tmp>>8);
  motor_can_send_data[7] = (uint8_t)(Torque_Tmp);

  return HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
}



/*解码函数*/
void decode_motor_measure(motor_t * motor, uint8_t * data)
{
  motor->angle_ecd = (data[0] << 8) | data[1];
  motor->speed_rpm = (data[2] << 8) | data[3];
  motor->current_raw = (data[4] << 8) | data[5];
  motor->temperate = data[6];
  return;
}

void decode_motor_measure_DM(motor_t * motor, uint8_t * data)
{
  motor->angle_ecd = (data[1] << 8) | data[2];
  motor->speed_rpm = (data[3] << 8) | data[4];
  motor->current_raw = (data[4] << 8) | data[5];
  motor->temperate = data[6];
  return;
}

/**
 * @brief can接收回调函数 用于解码
 *
 * @param hcan
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
  //标识符=0x200+ID
  if (rx_header.StdId == 0x201) {
    decode_motor_measure(&M3508_1, rx_data);
  }
  if (rx_header.StdId == 0x202) {
    decode_motor_measure(&M3508_2, rx_data);
  }
  if (rx_header.StdId == 0x203) {
    decode_motor_measure(&M3508_3, rx_data);
  }
  if (rx_header.StdId == 0x204) {
    decode_motor_measure(&M3508_4, rx_data);
  }
  if (rx_header.StdId == 0x0B ){
    decode_motor_measure(&J4310, rx_data);
  }
    if (rx_header.StdId == 0x302) {
    decode_motor_measure(&J4310, rx_data);
  }
    if (rx_header.StdId == 0x303) {
    decode_motor_measure(&J4310, rx_data);
  }
    if (rx_header.StdId == 0x304) {
    decode_motor_measure(&J4310, rx_data);
  }
}
