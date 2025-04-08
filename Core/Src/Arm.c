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
extern uint8_t Switch_flag;

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
double Arm_Posture[][4] = {{0.0,0.5,0.8,0.0},
                            {0.0,1.0,1.5,0.05},
                            {0.0,0.06,0.07,0.7},
    };
/**
 * @brief channel0 是 6500K灯珠亮度，channel1是3000K 灯珠亮度， 亮度范围 0-150
 */
uint8_t Light_Channel[2] = {0,0};

void Arm_Init()
{
    HAL_TIM_Base_Start_IT(&htim2);

    Arm_Motor_Enable();

    Arm_Motor_Pos_cmd(HOMING_POSTURE);//修改Pos数组的值防止比较失败
    //等待yaw pitch1 pitch2电机就位
    while(!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
        (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
        (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(HOMING_POSTURE);
    }

    HAL_Delay(500);

    //堵转回0
    for (uint8_t i = 0; i < 130; i++){
        cmd_motor(0x200,-700,0,0,0);
        HAL_Delay(3);
    }
    //阻塞主函数 直到M2006电机零点标记完成,堵转认为标记完成
    while(M2006_1.speed_rpm < -100){
        cmd_motor(0x200,-700,0,0,0);
        HAL_Delay(3);
    }

    HAL_TIM_Base_Start_IT(&htim5);//打开写在中断回调里的Pitch3电机PID控制
    angle = 0;
    last_angle = M2006_1.angle_ecd;
    angle+=5000; //防止从反方向转到100

    //ptich3就位
    Arm_Motor_Pos_cmd(BASE_POSTURE);
    //打开灯
    for(uint8_t i = 0; i < 4; i++){Arm_Light_cmd(6000,244);}
    HAL_Delay(300);
}

void Arm_Motor_Enable() //yaw pitch1 pitch2 使能
{
    //阻塞直到使能成功
    while(J4310_1.Err != 1){
        DM_Enable(0x101);
        HAL_Delay(3);
    }
    while(J4310_2.Err != 1){
        DM_Enable(0x102);
        HAL_Delay(3);
    }
    while(J4310_3.Err != 1){
        DM_Enable(0x103);
        HAL_Delay(3);
    }
    Enable_flag = 1;
}

void Arm_Motor_Disable() //yaw pitch1 pitch2 失能
{
    //阻塞直到失能成功
    while(J4310_1.Err != 0){
        DM_Disable(0x101);
        HAL_Delay(3);
    }
    while(J4310_2.Err != 0){
        DM_Disable(0x102);
        HAL_Delay(3);
    }
    while(J4310_3.Err != 0){
        DM_Disable(0x103);
        HAL_Delay(3);
    }
    Enable_flag = 0;
}

void Arm_Motor_Pos_cmd(uint8_t Posture)
{
    for(uint8_t i = 0; i < 4; i++){
        Pos[i] = Arm_Posture[Posture][i];
    }

    DM_SpeedPosition_cmd(&hcan1,0x101,Vel[0],Pos[0]);
    HAL_Delay(5);
    DM_SpeedPosition_cmd(&hcan1,0x102,Vel[1],Pos[1]);
    HAL_Delay(5);
    DM_SpeedPosition_cmd(&hcan1,0x103,Vel[2],Pos[2]);
    HAL_Delay(5);

    while(!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
    (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
    (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
        DM_SpeedPosition_cmd(&hcan1,0x101,Vel[0],Pos[0]);
        HAL_Delay(5);
        DM_SpeedPosition_cmd(&hcan1,0x102,Vel[1],Pos[1]);
        HAL_Delay(5);
        DM_SpeedPosition_cmd(&hcan1,0x103,Vel[2],Pos[2]);
        HAL_Delay(5);
    }

}

/**
 * @brief 计算指定色温指定亮度的对应channel值
 * @param Temperature 输入色温， 3000-6500
 * @param Light 输入亮度 0-70
 */
void Arm_Light_cmd(uint16_t Temperature,uint8_t Light)
{
    uint8_t channel1 = ((Temperature-3000)/3500.0)*Light;
    uint8_t channel2 = ((6500-Temperature)/3500.0)*Light;
    Light_Channel[0] = ((((channel1 > 148) ? 148 : channel1) < 0) ? 0 : (channel1 > 148) ? 148 : channel1);
    Light_Channel[1] = ((((channel2 > 148) ? 148 : channel2) < 0) ? 0 : (channel2 > 148) ? 148 : channel2);
    Light_cmd();
}

/**
 * @brief 机械臂关闭
 */
void Arm_Off()
{
    for(uint8_t i=0;i<4;i++){Arm_Light_cmd(6000,0);}
    Arm_Motor_Pos_cmd(OFF_POSTURE);
    while(!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
    (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
    (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(OFF_POSTURE);
    }
    HAL_Delay(500);

    //关闭pitch3
    cmd_motor(0x200,0,0,0,0);
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_TIM_Base_Stop_IT(&htim2);
    cmd_motor(0x200,0,0,0,0);

    Arm_Motor_Disable();
    HAL_Delay(300);
}

/**
 * @brief 开关初始化
 */
void Arm_Switch_Init()
{
    if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_SET) {
        Switch_flag = 0;
        for(uint8_t i=0;i<4;i++){Arm_Light_cmd(6000,0);}
    }else if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_RESET) {
        Switch_flag = 1;
        for(uint8_t i = 0; i < 4; i++){Arm_Light_cmd(6000,244);}
    }
}
