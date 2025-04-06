//
// Created by 22560 on 25-4-5.
//

#include "main.h"
#include "tim.h"
#include "Arm_Calc.h"
#include "can.h"
#include "Arm.h"
#include "PID.h"
#include "Communications.h"

extern DM_motor_t J4310_1;
extern DM_motor_t J4310_2;
extern DM_motor_t J4310_3;
extern uint8_t Enable_flag;
extern M2006_motor_t M2006_1;
extern int32_t last_angle;
extern int32_t angle;

/**
 * @brief 机械臂yaw pitch轴角度
 */
double Pos[4] = {0.0,0.0,0.0,0.0};
/**
 * @brief [0]为 yaw_target,[1]为 X_B ,[2] 为 Y_B，[3]为 pitch3_target
 */
double Arm_params_input[4] = {0.0,64.0,64.0};
/**
 * @brief 电机转动的最大速度
 */
double Vel[4] = {0.5,0.5,0.5,0.5};
/**
 * @brief 记录机械臂不同动作对应的电机角度q值
 */
double Arm_Posture[][4] = {{0.0,0.5,0.4,0.0},
    };

void Arm_Init()
{
    HAL_TIM_Base_Start_IT(&htim2);

    Arm_Motor_Enable();
    Arm_Motor_Pos_cmd(HOMING_POSTURE);

    HAL_Delay(3000);

    //阻塞主函数 直到M2006电机零点标记完成,按下key认为标记完成
    // if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
    // {
    //     HAL_Delay(15);//按键消抖
    //     // while(HAL_GPIO_ReadPin(GPIOA,KEY_Pin) == GPIO_PIN_SET) {}
    //     if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_RESET)
    //     {
    //         HAL_TIM_Base_Start_IT(&htim5);//打开写在中断回调里的Pitch3电机PID控制
    //         last_angle = M2006_1.angle_ecd;
    //         angle+=3000; //防止从反方向转到100
    //     }
    // }

    //堵转回0
    for (uint8_t i = 0; i < 130; i++)
    {
        cmd_motor(0x200,-700,0,0,0);
        HAL_Delay(3);
    }

    while(M2006_1.speed_rpm < -100)
    {
        cmd_motor(0x200,-700,0,0,0);
        HAL_Delay(3);
    }
    HAL_TIM_Base_Start_IT(&htim5);//打开写在中断回调里的Pitch3电机PID控制
    last_angle = M2006_1.angle_ecd;
    angle+=3000; //防止从反方向转到100

}

void Arm_Motor_Enable() //yaw pitch1 pitch2 使能
{
    //阻塞直到使能成功
    while(J4310_1.Err != 1)
    {
        DM_Enable(0x101);
        HAL_Delay(3);
    }
    while(J4310_2.Err != 1)
    {
        DM_Enable(0x102);
        HAL_Delay(3);
    }
    while(J4310_3.Err != 1)
    {
        DM_Enable(0x103);
        HAL_Delay(3);
    }
    Enable_flag = 1;
}

void Arm_Motor_Disable() //yaw pitch1 pitch2 失能
{
    //阻塞直到失能成功
    while(J4310_1.Err != 0)
    {
        DM_Disable(0x101);
        HAL_Delay(3);
    }
    while(J4310_2.Err != 0)
    {
        DM_Disable(0x102);
        HAL_Delay(3);
    }
    while(J4310_3.Err != 0)
    {
        DM_Disable(0x103);
        HAL_Delay(3);
    }
    Enable_flag = 0;
}

void Arm_Motor_Pos_cmd(uint8_t Posture)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        Pos[i] = Arm_Posture[Posture][i];
    }

    DM_SpeedPosition_cmd(&hcan1,0x101,Vel[0],Pos[0]);
    HAL_Delay(5);
    DM_SpeedPosition_cmd(&hcan1,0x102,Vel[1],Pos[1]);
    HAL_Delay(5);
    DM_SpeedPosition_cmd(&hcan1,0x103,Vel[2],Pos[2]);
    HAL_Delay(5);
}
