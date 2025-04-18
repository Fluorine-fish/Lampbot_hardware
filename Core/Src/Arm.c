//
// Created by 22560 on 25-4-5.
//

#include "main.h"
#include "tim.h"
#include "Arm_Calc.h"
#include "can.h"
#include "Arm.h"
#include "remote.h"
#include "Communications.h"
#include "math.h"

#include "Light.h"
#include "DM4310.h"
#include "M2006.h"

Light_TypeDef light1;
Posture_enum Arm_Posture_index;
extern DM4310_HandleTypeDef DM4310_1;
extern DM4310_HandleTypeDef DM4310_2;
extern DM4310_HandleTypeDef DM4310_3;
extern M2006_HandleTypeDef M2006_1;

extern uint8_t Enable_flag;
extern int32_t last_angle;
extern int32_t angle;
extern uint8_t Switch_flag;
extern RC_t RC;

/**
 * @brief 机械臂yaw pitch轴角度
 */
double Pos[4] = {0.0, 0.0, 0.0, 0.0};
/**
 * @brief [0]为 yaw_target,[1]为 X_B ,[2] 为 Y_B，[3]为 pitch3_target
 */
double Arm_params_input[4] = {0.0, 64.0, 64.0};
/**
 * @brief 电机转动的最大速度
 */
double Vel[4] = {1.5, 0.6, 0.6, 1500};
int16_t M2006_Max_Vel = 250;
/**
 * @brief 记录机械臂不同动作对应的电机角度q值
 */
double Arm_Posture[][4] = {
    {0.0, 0.5, 0.8, 0.0},
    {-0.5, 0.95, 0.95, 0.15},
    {0.0, 0.06, 0.07, 0.7},
    {-0.5, 0.95, 0.95, 0.15},
    {-0.5, 0.80, 0.75, 0.54},
    {-2.4, 0.70, 0.74, 1.20},

    {-0.5, 0.95, 0.45, 1.75},
    {-1.35, 0.95, 0.45, 1.75},
    {-1.05, 0.95, 0.45, 1.75},
    {-1.65, 0.95, 0.45, 1.75},
    {1.85, 0.95, 0.45, 1.75},
    {2.15, 0.95, 0.45, 1.75},
    {1.55, 0.95, 0.45, 1.75},
    {1.05, 0.95, 0.45, 1.75},
    {0.75, 0.95, 0.45, 1.75},
    {1.35, 0.95, 0.45, 1.75},
    {1.05, 0.95, 0.45, 1.75},
    {2.25, 1, 0.25, 0.7},
    {2.70, 1, 0.20, 0.55},

    {-0.5, 1.65, 1.60, 0.35},
};
/**
 * @brief 记录机械臂把自己关掉需要的动作
 */
double Turn_Itself_Off_Posture[][4] = {
    {-0.5, 0.95, 0.45, 1.75},
    {-1.35, 0.95, 0.45, 1.75},
    {-1.25, 0.95, 0.45, 1.75},
    {-1.45, 0.95, 0.45, 1.75},
    {1.65, 0.95, 0.45, 1.75},
    {1.75, 0.95, 0.45, 1.75},
    {1.55, 0.95, 0.45, 1.75},
    {1.05, 0.95, 0.45, 1.75},
    {0.95, 0.95, 0.45, 1.75},
    {1.15, 0.95, 0.45, 1.75},
    {1.05, 0.95, 0.45, 1.75},
    {2.25, 1, 0.25, 0.7},
    {2.85, 1, 0.20, 0.55},
};
/**
 * @brief channel0 是 6500K灯珠亮度，channel1是3000K 灯珠亮度， 亮度范围 0-150
 */
uint16_t Temperature = 6000;
uint16_t Light = 100;

void Arm_Start() {
    HAL_TIM_Base_Start_IT(&htim2);

    Light_Init(&light1, &hcan1);

    Arm_Motor_Enable();

    Arm_Motor_Pos_cmd(Homing_Posture); //修改Pos数组的值防止比较失败
    //等待yaw pitch1 pitch2电机就位
    while (!((DM4310_1.position - Pos[0] <= 0.1) && (DM4310_1.position - Pos[0] >= -0.1) &&
        (DM4310_2.position - Pos[1] <= 0.1) && (DM4310_2.position - Pos[1] >= -0.1) &&
        (DM4310_3.position - Pos[2] <= 0.1) && (DM4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(Homing_Posture);
    }

    HAL_Delay(500);

    //堵转回0
    for (uint8_t i = 0; i < 130; i++) {
        M2006_Crtl_Currency(&M2006_1, -700);
        HAL_Delay(3);
    }
    //阻塞主函数 直到M2006电机零点标记完成,堵转认为标记完成
    while (M2006_1.speed_rpm < -100) {
        M2006_Crtl_Currency(&M2006_1, -700);
        HAL_Delay(3);
    }

    HAL_TIM_Base_Start_IT(&htim5); //打开写在中断回调里的Pitch3电机PID控制
    angle = 0;
    last_angle = M2006_1.angle_ecd;
    angle += 5000; //防止从反方向转到100

    //ptich3就位
    Arm_Motor_Pos_cmd(Base_Posture);
    //打开灯
    Arm_Light_slow_ON();
    for (uint8_t i = 0; i < 4; i++) { Light_Ctrl(&light1, Temperature, 150); }
}

void Arm_Quick_Start() {
    HAL_TIM_Base_Start_IT(&htim2);

    Light_Init(&light1, &hcan1);
    Arm_Motor_Enable();

    Arm_Motor_Pos_cmd(Homing_Posture); //修改Pos数组的值防止比较失败
    //等待yaw pitch1 pitch2电机就位
    while (!((DM4310_1.position - Pos[0] <= 0.1) && (DM4310_1.position - Pos[0] >= -0.1) &&
        (DM4310_2.position - Pos[1] <= 0.1) && (DM4310_2.position - Pos[1] >= -0.1) &&
        (DM4310_3.position - Pos[2] <= 0.1) && (DM4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(Homing_Posture);
        }

    HAL_Delay(500);

    //堵转回0
    for (uint8_t i = 0; i < 130; i++) {
        M2006_Crtl_Currency(&M2006_1, -700);
        HAL_Delay(3);
    }
    //阻塞主函数 直到M2006电机零点标记完成,堵转认为标记完成
    while (M2006_1.speed_rpm < -100) {
        M2006_Crtl_Currency(&M2006_1, -700);
        HAL_Delay(3);
    }

    HAL_TIM_Base_Start_IT(&htim5); //打开写在中断回调里的Pitch3电机PID控制
    angle = 0;
    last_angle = M2006_1.angle_ecd;
    angle += 5000; //防止从反方向转到100

    //ptich3就位
    Arm_Motor_Pos_cmd(Base_Posture);
    //打开灯
    for (uint8_t i = 0; i < 4; i++) { Light_Ctrl(&light1, Temperature, 150); }
}

void Arm_Motor_Enable() //yaw pitch1 pitch2 使能
{
    //阻塞直到使能成功
    while (DM4310_1.Err != 1) {
        DM_Enable(&DM4310_1);
        HAL_Delay(3);
    }
    while (DM4310_3.Err != 1) {
        DM_Enable(&DM4310_3);
        HAL_Delay(3);
    }
    while (DM4310_2.Err != 1) {
        DM_Enable(&DM4310_2);
        HAL_Delay(3);
    }
    Enable_flag = 1;
}

void Arm_Motor_Disable() //yaw pitch1 pitch2 失能
{
    //阻塞直到失能成功
    while (DM4310_1.Err != 0) {
        DM_Disable(&DM4310_1);
        HAL_Delay(3);
    }
    while (DM4310_2.Err != 0) {
        DM_Disable(&DM4310_2);
        HAL_Delay(3);
    }
    while (DM4310_3.Err != 0) {
        DM_Disable(&DM4310_3);
        HAL_Delay(3);
    }
    Enable_flag = 0;
}

void Arm_Motor_Pos_cmd(uint8_t Posture) {
    for (uint8_t i = 0; i < 4; i++) {
        Pos[i] = Arm_Posture[Posture][i];
    }

    DM4310_Ctrl(&DM4310_1, Pos[0], Vel[0]);
    HAL_Delay(5);
    DM4310_Ctrl(&DM4310_2, Pos[1], Vel[1]);
    HAL_Delay(5);
    DM4310_Ctrl(&DM4310_3, Pos[2], Vel[2]);
    HAL_Delay(5);

    //确保到位
    if (Posture != Remote_Posture) {
        while (!((DM4310_1.position - Pos[0] <= 0.1) && (DM4310_1.position - Pos[0] >= -0.1) &&
            (DM4310_2.position - Pos[1] <= 0.1) && (DM4310_2.position - Pos[1] >= -0.1) &&
            (DM4310_3.position - Pos[2] <= 0.1) && (DM4310_3.position - Pos[2] >= -0.1))) {
            DM4310_Ctrl(&DM4310_1, Pos[0], Vel[0]);
            HAL_Delay(5);
            DM4310_Ctrl(&DM4310_2, Pos[1], Vel[1]);
            HAL_Delay(5);
            DM4310_Ctrl(&DM4310_3, Pos[2], Vel[2]);
            HAL_Delay(5);
        }
    }
}


/**
 * @brief 机械臂关闭 包含缓慢关灯
 */
void Arm_Off() {
    Arm_Light_slow_OFF();
    for (uint8_t i = 0; i < 4; i++) { Light_Ctrl(&light1, Temperature, 0); }
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.7; }
    Vel[0] = 1.5;
    Arm_Motor_Pos_cmd(Off_Posture);
    while (!((DM4310_1.position - Pos[0] <= 0.1) && (DM4310_1.position - Pos[0] >= -0.1) &&
        (DM4310_2.position - Pos[1] <= 0.1) && (DM4310_2.position - Pos[1] >= -0.1) &&
        (DM4310_3.position - Pos[2] <= 0.1) && (DM4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(Off_Posture);
    }
    HAL_Delay(500);

    //关闭pitch3
    M2006_Crtl_Currency(&M2006_1, 0);
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_TIM_Base_Stop_IT(&htim2);
    M2006_Crtl_Currency(&M2006_1, 0);

    Arm_Motor_Disable();
    HAL_Delay(300);
}

void Arm_Quick_Off() {
    for (uint8_t i = 0; i < 4; i++) { Light_Ctrl(&light1, Temperature, 0); }
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.7; }
    Vel[0] = 1.5;
    Arm_Motor_Pos_cmd(Off_Posture);
    while (!((DM4310_1.position - Pos[0] <= 0.1) && (DM4310_1.position - Pos[0] >= -0.1) &&
        (DM4310_2.position - Pos[1] <= 0.1) && (DM4310_2.position - Pos[1] >= -0.1) &&
        (DM4310_3.position - Pos[2] <= 0.1) && (DM4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(Off_Posture);
    }
    HAL_Delay(500);

    //关闭pitch3
    M2006_Crtl_Currency(&M2006_1, 0);
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_TIM_Base_Stop_IT(&htim2);
    M2006_Crtl_Currency(&M2006_1, 0);

    Arm_Motor_Disable();
    HAL_Delay(300);
}

/**
 * @brief 开关初始化
 */
void Arm_Switch_Init() {
    if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_SET) {
        Switch_flag = 0;
    }
    else if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == GPIO_PIN_RESET) {
        Switch_flag = 1;
    }
}

/**
 * @breif 开启遥控器模式: ch0->yaw;ch1->pitch1;ch2->pitch2；ch3->pitch3
 * yaw: -2.256194 ~ 2.256194
 * pitch1: 0.1 ~ 3.0415927
 * pitch2: 0.1 ~ 3.0415927
 * pitch3: 0.1 ~ 2.1340214
 * Temperature: 4000~5500
 */
void Arm_Remote_Mode() {
    //在遥控器模式下再进行执行
    if (RC.s1 == 1 && RC.s2 == 1) {
        //提升响应速度
        Vel[0] = 2.5;
        Vel[1] = 1;
        Vel[2] = 1;
        Vel[3] = 1;

        double temp_Pos[4];
        uint16_t temp_Temperature = 6000;
        for (uint8_t i = 0; i < 4; i++) { temp_Pos[i] = Arm_Posture[Remote_Posture][i]; }
        temp_Temperature = Temperature;

        if (RC.ch0 >= 20) {
            temp_Pos[0] += 0.05;
        }
        else if (RC.ch0 <= -200) {
            temp_Pos[0] -= 0.1;
        }
        if (RC.ch1 >= 20) {
            temp_Pos[1] += 0.05;
        }
        else if (RC.ch1 <= -200) {
            temp_Pos[1] -= 0.1;
        }
        if (RC.ch2 >= 20) {
            temp_Pos[2] += 0.05;
        }
        else if (RC.ch2 <= -200) {
            temp_Pos[2] -= 0.05;
        }
        if (RC.ch3 >= 20) {
            temp_Pos[3] += 0.05;
        }
        else if (RC.ch3 <= -200) {
            temp_Pos[3] -= 0.05;
        }
        if (RC.wheel >= 20) {
            temp_Temperature -= 100;
        }
        else if (RC.wheel <= -200) {
            temp_Temperature += 100;
        }

        //传入参数限幅
        temp_Pos[0] = clamp(temp_Pos[0], -3.1315927, 3.1315927);
        temp_Pos[1] = clamp(temp_Pos[1], 0.1, 3.0415927);
        temp_Pos[2] = clamp(temp_Pos[2], 0.1, 3.0415927);
        temp_Pos[3] = clamp(temp_Pos[3], 0.1, 2.1340214);
        temp_Temperature = ((((temp_Temperature > 6000) ? 6000 : temp_Temperature) < 3500)
                                ? 3500
                                : (temp_Temperature > 6000)
                                ? 6000
                                : temp_Temperature);

        //Pos 赋值
        for (uint8_t i = 0; i < 4; i++) { Arm_Posture[Remote_Posture][i] = temp_Pos[i]; }
        Temperature = temp_Temperature;

        Arm_Motor_Pos_cmd(Remote_Posture);
        Light_Ctrl(&light1, Temperature,Light);
    }
    else {
        Vel[0] = 1.5;
        Vel[1] = 0.7;
        Vel[2] = 0.7;
        Vel[3] = 0.7;

        for (uint8_t i = 0; i < 4; i++) { Arm_Posture[Remote_Posture][i] = Arm_Posture[Base_Posture][i]; }
    }
}

/**
 * @brief 缓启动灯光
 */
void Arm_Light_slow_ON() {
    float cnt = 0.0;
    for (int i = 0; i < 100; i++) {
        cnt = i * 0.015; // 计算当前 cnt 值
        Light_Ctrl(&light1, Temperature, 150 * sin(cnt));
        HAL_Delay(50);
    }
    Light_Ctrl(&light1, Temperature, 150); // 循环结束后设置为最大值
}

/**
 * @brief 缓关闭灯光
 */
void Arm_Light_slow_OFF() {
    float cnt = 0.0;
    for (int i = 40; i < 100; i++) {
        cnt = 3.1415927 / 2.0 + i * 0.015; // 计算当前 cnt 值
        Light_Ctrl(&light1, Temperature, 150 * sin(cnt));
        HAL_Delay(50);
    }
    Light_Ctrl(&light1, Temperature, 0); // 循环结束后设置为最大值
}

void Arm_Light_Remote() {
    uint16_t temp_Temperature = Temperature;
    int16_t temp_Light = Light;
    temp_Temperature = Temperature;
    if (RC.ch1 >= 20) {
        temp_Temperature -= 100;
    }
    else if (RC.ch1 <= -20) {
        temp_Temperature += 100;
    }
    if (RC.ch3 >= 20) {
        temp_Light += 10;
    }
    else if (RC.ch3 <= -20) {
        temp_Light -= 10;
    }

    temp_Temperature = ((((temp_Temperature > 6000) ? 6000 : temp_Temperature) < 3500)
                            ? 3500
                            : (temp_Temperature > 6000)
                            ? 6000
                            : temp_Temperature);
    temp_Light = ((((temp_Light > 244) ? 244 : temp_Light) < 0) ? 0 : (temp_Light > 244) ? 244 : temp_Light);

    Temperature = temp_Temperature;
    Light = temp_Light;

    Light_Ctrl(&light1, Temperature, Light);
}

void Arm_Remind_Sitting() {
    Arm_Motor_Pos_cmd(Remind_Sitting_Posture);
    HAL_Delay(200);
    Pos[3] += 0.1;
    HAL_Delay(400);
    Pos[3] -= 0.1;
    HAL_Delay(400);
    Arm_Motor_Pos_cmd(Base_Posture);
    HAL_Delay(4000);
}

void Arm_Looking_Forward() {
    Arm_Light_slow_OFF();
    Arm_Motor_Pos_cmd(Remind_Looking_Forward_Posture);
    HAL_Delay(200);
    Pos[3] += 0.1;
    HAL_Delay(1000);
    Pos[3] -= 0.1;
    HAL_Delay(8000);
    Arm_Motor_Pos_cmd(Base_Posture);
    Arm_Light_slow_ON();
}

void Arm_Back() {
    Arm_Motor_Pos_cmd(Base_Posture);
    for (uint8_t i = 0; i < 4; i++) { Pos[i] = Arm_Posture[Base_Posture][i]; }
    for (uint8_t i = 0; i < 4; i++) { Arm_Posture[Remote_Posture][i] = Arm_Posture[Base_Posture][i]; }
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.6; }
    Vel[0] = 1.5;
    Light_Ctrl(&light1, Temperature, Light);
}

void Arm_Turn_Itself_Off() {
    Arm_Motor_Pos_cmd(Base_Posture);
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.8; }
    Vel[0] = 1.3;
    HAL_Delay(200);
    //执行自动关机程序
    Arm_Motor_Pos_cmd(Turn_Itself_Off_1);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_2);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_3);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_4);
    HAL_Delay(600);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_5);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_6);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_7);
    HAL_Delay(600);

    Vel[0] = 0.7;
    Arm_Motor_Pos_cmd(Turn_Itself_Off_8);
    HAL_Delay(600);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_9);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_10);
    HAL_Delay(1000);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_11);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_12);
    HAL_Delay(800);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_13);
    HAL_Delay(200);
}

void Arm_Quick_Turn_Itself_Off() {
    Arm_Motor_Pos_cmd(Base_Posture);
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.8; }
    Vel[0] = 1.3;
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_12);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_13);
    HAL_Delay(200);
}

void Arm_Light_Tracing_Present() {
    Arm_Motor_Pos_cmd(Light_Tracing_Posture);
    HAL_Delay(3000);
    Arm_Motor_Pos_cmd(Base_Posture);
    HAL_Delay(3000);
}

/**
 * 拨杆开关从上到下是 1，3，2
 */
void Arm_Task() {
    if (RC.s1 == 1 && RC.s2 == 1) {
        Arm_Remote_Mode();
    }
    else if (RC.s1 == 1 && RC.s2 == 3) {
        Arm_Light_Remote();
    }
    else if (RC.s1 == 3 && RC.s2 == 1) {
        Arm_Remind_Sitting();
    }
    else if (RC.s1 == 3 && RC.s2 == 2) {
        Arm_Light_Tracing_Present();
    }
    else if (RC.s1 == 3 && RC.s2 == 3) {
        Arm_Looking_Forward();
    }
    else if (RC.s1 == 2 && RC.s2 == 2) {
        Arm_Turn_Itself_Off();
    }
    else if (RC.s1 == 2 && RC.s2 == 3) {
        Arm_Quick_Turn_Itself_Off();
    }
    else {
        Arm_Back();
    }
}
