//
// Created by 22560 on 25-3-18.
//

#include "stm32f407xx.h"
#include "can.h"
#include "main.h"

//大疆电机数据存储结构体
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
  int16_t speed_rpm;
  int16_t torque;
}M2006_motor_t;

typedef struct
{
  int p_int,v_int,t_int;						//这里可根据电机数目自行修改，读取三个电机的位置、速度、转矩
  float position,velocity,torque;
  int16_t State;
}DM_motor_t;

motor_t motor_6020={0,0,0,0};
motor_t M3508_1={0,0,0,0};
motor_t M3508_2={0,0,0,0};
motor_t M3508_3={0,0,0,0};
motor_t M3508_4={0,0,0,0};

DM_motor_t J4310_1={0,0,0,0,0,0,0};
DM_motor_t J4310_2={0,0,0,0,0,0,0};
DM_motor_t J4310_3={0,0,0,0,0,0,0};
DM_motor_t J4310_4={0,0,0,0,0,0,0};
M2006_motor_t M2006_1={0,0,0};

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
 * @brief 达妙电机速度位置模式报文
 *
 * @param stdid
 * @param vel fp16类型的速度值
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DM_SpeedPosition_cmd(CAN_HandleTypeDef *hacn,uint32_t stdid, float vel, float pos){
  uint32_t send_mail_box;
  motor_tx_message.StdId = stdid;
  motor_tx_message.IDE = CAN_ID_STD; //使用标准帧格式
  motor_tx_message.RTR = CAN_RTR_DATA; //数据帧类型
  motor_tx_message.DLC = 0x08;

  uint8_t *pbuf,*vbuf;
  pbuf=(uint8_t*)&pos;
  vbuf=(uint8_t*)&vel;
  motor_can_send_data[0] =*pbuf;
  motor_can_send_data[1] = *(pbuf+1);
  motor_can_send_data[2] = *(pbuf+2);
  motor_can_send_data[3] = *(pbuf+3);
  motor_can_send_data[4] = *vbuf;
  motor_can_send_data[5] = *(vbuf+1);
  motor_can_send_data[6] = *(vbuf+2);
  motor_can_send_data[7] = *(vbuf+3);

  return HAL_CAN_AddTxMessage(hacn, &motor_tx_message, motor_can_send_data, &send_mail_box);
}

//达妙相关函数；
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

/*解码函数*/
void decode_motor_measure(motor_t * motor, uint8_t * data)
{
  motor->angle_ecd = (data[0] << 8) | data[1];
  motor->speed_rpm = (data[2] << 8) | data[3];
  motor->current_raw = (data[4] << 8) | data[5];
  motor->temperate = data[6];
}

void decode_motor_measure_M2006(M2006_motor_t * motor, uint8_t * data)
{
  motor->angle_ecd = (data[0] << 8) | data[1];
  motor->speed_rpm = (data[2] << 8) | data[3];
  motor->torque = (data[4] << 8) | data[5];
}

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

void decode_motor_measure_DM(DM_motor_t * motor, uint8_t * data)
{
  motor->p_int=(data[1]<<8)|data[2];
  motor->v_int=(data[3]<<4)|(data[4]>>4);
  motor->t_int=((data[4]&0xF)<<8)|data[5];
  motor->position = uint_to_float(motor->p_int, -12.5, 12.5, 16); // (-12.5,12.5)
  motor->velocity = uint_to_float(motor->v_int, -45, 45, 12); // (-45.0,45.0)
  motor->torque = uint_to_float(motor->t_int, -18, 18, 12); // (-18.0,18.0)
  motor->State=(data[0] & 0x0F);
}

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
  /// Converts a float to an unsigned int, given range and number of bits///
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
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
  if (rx_header.StdId == 0x014) {
    decode_motor_measure_DM(&J4310_4, rx_data);
  }
}
