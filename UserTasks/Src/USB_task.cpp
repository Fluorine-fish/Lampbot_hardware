//
// Created by 22560 on 25-4-18.
//

#include "USB_task.h"
#include "usbd_cdc_if.h"

Command_FIFO cmd_FIFO;

class USB_Data {
public:
    uint8_t CPP_myUSBRxData[64];
    uint16_t CPP_myUSBRxNum;
    uint8_t CPP_myUSBTxData[64];
    uint16_t CPP_myUSBTxNum;
    SerialPacket_t msg_from_minipc;
    SerialPacket_t msg_to_minipc;

    void CPP_USBData_Init(void);
    void CPP_USBData_Process(void);
    void CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendMsg(char isOn,
                                   char isLightOn,
                                   uint32_t Light,
                                   uint32_t Temprature);
};

/**
 * @brief 初始化接收来自上位机数据的帧格式配置
 */
void USB_Data::CPP_USBData_Init() {
    msg_from_minipc.start = 's';
    msg_from_minipc.end = 'e';
    msg_from_minipc.datatype = 0xA0; // 上位机发送数据类型
    msg_from_minipc.command = 0xFF;  // 命令字，0xFF表示无效命令
    for (int i = 0; i < 8; i++) {
        msg_from_minipc.data[i] = 0; // 初始化数据域 上位机数据域留空
    }

    msg_to_minipc.start = 's';
    msg_to_minipc.end = 'e';
    msg_to_minipc.datatype = 0xB0; // 下位机发送数据类型
    msg_to_minipc.command = 0xFF;  // 命令字，0xFF表示无效命令
    msg_to_minipc.data[0] = 1;     //默认开机
    msg_to_minipc.data[1] = 1;     //初始化后是开灯状态
    msg_to_minipc.data[2] = 500;   //初始亮度
    msg_to_minipc.data[3] = 5300;  //初始色温
    for (int i = 4; i < 8; i++) {
        msg_from_minipc.data[i] = 0; // 初始化数据域 下位机剩余数据域留空
    }

    //FIFO 初始化
    cmd_FIFO.front = 0; // 队列头指针
    cmd_FIFO.rear = 0;  // 队列尾指针
    for (char i = 0; i < 20; i++) {
        cmd_FIFO.queue[i] = 0; // 初始化队列
    }
}

/**
 * @brief 处理数据
 */
void USB_Data::CPP_USBData_Process(void) {
    if (CPP_myUSBRxNum) {
        if (CPP_myUSBRxData[1] == 0xA0) {
            //满足上位机发送数据协议的处理方法
            msg_from_minipc.start = CPP_myUSBRxData[0];
            msg_from_minipc.datatype = CPP_myUSBRxData[1];
            msg_from_minipc.command = CPP_myUSBRxData[2];
            /*上位机不向下位机发送数据包*/
            msg_from_minipc.end = CPP_myUSBRxData[31];

            char command = msg_from_minipc.command;
            if (command != 0xFF) {
                if (command == 0xAF) { //0xAF用来测试出队
                    char cmd;
                    deQueue(&cmd_FIFO, &cmd);
                }else {
                    enQueue(&cmd_FIFO, command); //将命令入队
                }
            }else {
                uint8_t msg_buf[32] = {0};
                msg_buf[0] = 'E'; //Error开头
                for (char i = 0; i < CPP_myUSBRxNum && i < 32; i++) {
                    msg_buf[i + 1] = CPP_myUSBRxData[i]; //将接收到的数据复制到msg_buf
                }
                uint32_t msg_len = CPP_myUSBRxNum + 1; //加上Error开头的字节
                CPP_USBData_SendData(msg_buf, &msg_len);
            }

            memset(CPP_myUSBRxData, 0, 64); //数据处理后清空缓存区
            CPP_myUSBRxNum = 0;             //有利于判断，置0
        }
        else {
            //如果不是协议约定的格式 就原样将数据发回，并且以Error开头
            uint8_t msg_buf[32] = {0};
            msg_buf[0] = 'E'; //Error开头
            for (char i = 0; i < CPP_myUSBRxNum && i < 32; i++) {
                msg_buf[i + 1] = CPP_myUSBRxData[i]; //将接收到的数据复制到msg_buf
            }
            uint32_t msg_len = CPP_myUSBRxNum + 1; //加上Error开头的字节
            CPP_USBData_SendData(msg_buf, &msg_len);
        }
        memset(CPP_myUSBRxData, 0, 64); //数据处理后清空缓存区
        CPP_myUSBRxNum = 0;             //有利于判断，置0
    }
}

void USB_Data::CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBRxData, 0, 64);     //清空缓存区
    memcpy(CPP_myUSBRxData, Buf, *Len); //接收的数据复制到缓存区
    CPP_myUSBRxNum = *Len;              //复制字节数
}

void USB_Data::CPP_USBData_SendMsg(char isOn,
                                   char isLightOn,
                                   uint32_t Light,
                                   uint32_t Temprature) {
    msg_to_minipc.start = 's';
    msg_to_minipc.end = 'e';
    msg_to_minipc.datatype = 0xB0; // 下位机发送数据类型
    msg_to_minipc.data[0] = isOn; //是否开机
    msg_to_minipc.data[1] = isLightOn; //是否开灯
    msg_to_minipc.data[2] = Light; //光照强度
    msg_to_minipc.data[3] = Temprature; //色温
    for (int i = 4; i < 8; i++) {
        msg_to_minipc.data[i] = 0; //下位机剩余数据域留空
    }

    CPP_myUSBTxNum = 32;
    CPP_myUSBTxData[0] = msg_to_minipc.start;
    CPP_myUSBTxData[1] = msg_to_minipc.datatype;

    // 将数据域转换为字节数组
    for (char i = 0; i < 8; i++) {
        for (char j = 0; j < 4; j++) {
            CPP_myUSBTxData[i * 4 + j + 2] = *((char*)(&msg_to_minipc.data[i]) + j);
        }
    }

    for (char i = 27; i < 30; i++) {
        //中间留空
        CPP_myUSBTxData[i] = 0x00;
    }
    CPP_myUSBTxData[31] = msg_to_minipc.end;

    CDC_Transmit_FS(CPP_myUSBTxData, CPP_myUSBTxNum);
    memset(CPP_myUSBTxData, 0, 64); //数据处理后清空缓存区
    CPP_myUSBTxNum = 0;             //有利于判断，置0
}


void USB_Data::CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBTxData, 0, 64);
    memcpy(CPP_myUSBTxData, Buf, *Len);
    CPP_myUSBTxNum = *Len;
    CDC_Transmit_FS(Buf, *Len);
    memset(CPP_myUSBTxData, 0, 64);
}

USB_Data USB_Data1;

void CPP_USB_Task() {
    USB_Data1.CPP_USBData_Process();
    // USB_Data1.CPP_USBData_SendMsg(_todo.posture_reminder, _todo.eye_reminder);
}

extern "C" {
void USB_Task() {
    CPP_USB_Task();
}

void USBData_Process() {
    USB_Data1.CPP_USBData_Process();
}

void USBData_GetData(uint8_t* Buf, uint32_t* Len) {
    USB_Data1.CPP_USBData_GetData(Buf, Len);
}

void USBData_SendData(uint8_t* Buf, uint32_t* Len) {
    USB_Data1.CPP_USBData_SendData(Buf, Len);
}

void USBData_init() {
    USB_Data1.CPP_USBData_Init();
}
}
