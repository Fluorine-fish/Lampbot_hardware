//
// Created by 22560 on 25-4-18.
//

#ifndef USB_TASK_H
#define USB_TASK_H
#include "main.h"

typedef struct {
    uint8_t queue[20]; // 命令消息循环队列
    uint8_t front; // 队列头指针
    uint8_t rear; // 队列尾指针
} Command_FIFO;

typedef struct {
    char start;     //0 帧头取 's'
    char type;      //1 消息类型：上->下：0xA3 下->上：0xB3
    char light_on;
    char light_off;
    char light_brighter;
    char light_dimmer;
    char posture_reminder;
    char eye_reminder;
    char end;       //31 帧尾取'e'
} usb_msg_t;

typedef struct {
    char light_on;
    char light_off;
    char light_brighter;
    char light_dimmer;
    char posture_reminder;
    char eye_reminder;
} arm_todo_t;

typedef struct {
    char start;  // 0 帧头，取 's'
    char datatype; // 1 消息类型
    char command;  // 2 命令字
    uint32_t data[8]; // 3 - 26 数据域，最多存储8个float类型数据
     // 27 - 30 预留空位
    char end;    // 31 帧尾，取 'e'
} SerialPacket_t;

/***用于操作命令消息队列的函数***/

/** @brief 判断队列是否为空
 * @param q 命令消息队列指针
 * @return 1表示为空,0表示不为空
 */
inline char isEmpty(Command_FIFO *q) {
    if (q->front == q->rear) {
        return 1; // 队列为空
    }else {
        return 0; // 队列不为空
    }
};
/**
 * @brief 判断队列是否满
 * @param q 命令消息队列指针
 * @return 1表示为满,0表示不为满
 */
inline char isFull(Command_FIFO *q) {
    if ( (q->rear + 1) % 20 == q->front) {
        return 1; // 队列已满
    } else {
        return 0; // 队列未满
    }
}
/**
 * @brief 队列入队
 * @param q 命令消息队列指针
 * @param command 入队的命令
 * @return -1表示失败 0表示成功
 */
inline char enQueue(Command_FIFO *q, char command) {
    if (isFull(q)) {
        return -1;
    } else {
        q->queue[q->rear] = command; // 先写入数据
        q->rear = (q->rear + 1) % 20; // 再更新队尾指针
        return 0; // 成功入队
    }
}
/**
 * @brief 队列出队
 * @param q 命令队列指针
 * @param command 用于接收读出命令的指针
 * @return -1表示失败,0表示成功
 */
inline char deQueue(Command_FIFO *q, char *command) {
    if (isEmpty(q)) {
        return -1; // 队列为空，无法出队
    } else {
        *command = q->queue[q->front]; // 先读取数据
        q->front = (q->front + 1) % 20; // 再更新队头指针
        return 0; // 成功出队
    }
}

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief USB任务函数
 * @note 需要塞到循环中执行，也可以塞在定时器中断中执行
 */
void USB_Task();

void USBData_Process();

void USBData_GetData(uint8_t* Buf, uint32_t* Len);

void USBData_SendData(uint8_t* Buf, uint32_t* Len);

void USBData_init();
#ifdef __cplusplus
}
#endif

#endif //USB_TASK_H
