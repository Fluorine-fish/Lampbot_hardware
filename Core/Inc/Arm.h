//
// Created by 22560 on 25-4-5.
//

#ifndef ARM_H
#define ARM_H

#include "main.h"
#include "Light.h"

/*---------------------------------一些初始值声明-------------------------------*/
#define Original_light (500) //默认光照强度
#define Original_temperature (5300) //默认色温

/*---------------------------------------------------------------------------*/


extern double Arm_Posture[][4];
extern uint8_t Light_Channel[2];
extern Light_TypeDef light1;

typedef enum {
    Homing_Posture = 0,                 //init状态，默认使能后等待pitch3归位的状态
    Base_Posture = 1,                   //base状态，所有零点初始化结束后的状态
    Off_Posture = 2,                    //off状态 等待电机使能的关机状态
    Remote_Posture = 3,                 //remote状态 借助遥控器传回的数据控制姿态位置
    Remind_Sitting_Posture = 4,         //提醒坐姿状态
    Remind_Looking_Forward_Posture = 5, //提醒远眺姿势
    Light_Tracing_Posture = 19,         //展示光照跟随动作
    Book_Follow_Posture = 20,           //利用机械臂解算计算坐标运动模式

    //关闭自己的开关的运动路径
    Turn_Itself_Off_1 = 6,
    Turn_Itself_Off_2 = 7,
    Turn_Itself_Off_3 = 8,
    Turn_Itself_Off_4 = 9,
    Turn_Itself_Off_5 = 10,
    Turn_Itself_Off_6 = 11,
    Turn_Itself_Off_7 = 12,
    Turn_Itself_Off_8 = 13,
    Turn_Itself_Off_9 = 14,
    Turn_Itself_Off_10 = 15,
    Turn_Itself_Off_11 = 16,
    Turn_Itself_Off_12 = 17,
    Turn_Itself_Off_13 = 18,
} Posture_enum;

void Arm_Motor_Enable();
void Arm_Motor_Disable();
void Arm_Start();
void Arm_Quick_Start();
void Arm_Motor_Pos_cmd(uint8_t Posture);
void Arm_Off();
void Arm_Switch_Init();
void Arm_Remote_Mode();
void Arm_Task();
void Arm_Light_slow_ON();
void Arm_Light_slow_OFF();
void Arm_Light_Remote();
void Arm_Remind_Sitting();
void Arm_Looking_Forward();
void Arm_Back();
void Arm_Quick_Off();
void Arm_Turn_Itself_Off();
void Arm_Quick_Turn_Itself_Off();
void Arm_Light_Tracing_Present();
void Arm_Book_Follow();

#endif //ARM_H
