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

extern DM_motor_t J4310_1;
extern DM_motor_t J4310_2;
extern DM_motor_t J4310_3;
extern uint8_t Enable_flag;
extern M2006_motor_t M2006_1;
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
    {0.0, 1.0, 1.5, 0.05},
    {-0.5, 0.80, 0.75, 0.54},
    {-2.4, 0.70, 0.74, 1.20},
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
    {3.14, 1, 0.25, 0.7},
};
/**
 * @brief channel0 是 6500K灯珠亮度，channel1是3000K 灯珠亮度， 亮度范围 0-150
 */
uint8_t Light_Channel[2] = {0, 0};
uint16_t Temperature = 6000;
uint16_t Light = 244;

void Arm_Init() {
    HAL_TIM_Base_Start_IT(&htim2);

    Arm_Motor_Enable();

    Arm_Motor_Pos_cmd(HOMING_POSTURE); //修改Pos数组的值防止比较失败
    //等待yaw pitch1 pitch2电机就位
    while (!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
        (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
        (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(HOMING_POSTURE);
    }

    HAL_Delay(500);

    //堵转回0
    for (uint8_t i = 0; i < 130; i++) {
        cmd_motor(0x200, -700, 0, 0, 0);
        HAL_Delay(3);
    }
    //阻塞主函数 直到M2006电机零点标记完成,堵转认为标记完成
    while (M2006_1.speed_rpm < -100) {
        cmd_motor(0x200, -700, 0, 0, 0);
        HAL_Delay(3);
    }

    HAL_TIM_Base_Start_IT(&htim5); //打开写在中断回调里的Pitch3电机PID控制
    angle = 0;
    last_angle = M2006_1.angle_ecd;
    angle += 5000; //防止从反方向转到100

    //ptich3就位
    Arm_Motor_Pos_cmd(BASE_POSTURE);
    //打开灯
    Arm_Light_slow_ON();
    for (uint8_t i = 0; i < 4; i++) { Arm_Light_cmd(Temperature, 244); }
}

void Arm_Motor_Enable() //yaw pitch1 pitch2 使能
{
    //阻塞直到使能成功
    while (J4310_1.Err != 1) {
        DM_Enable(0x101);
        HAL_Delay(3);
    }
    while (J4310_2.Err != 1) {
        DM_Enable(0x102);
        HAL_Delay(3);
    }
    while (J4310_3.Err != 1) {
        DM_Enable(0x103);
        HAL_Delay(3);
    }
    Enable_flag = 1;
}

void Arm_Motor_Disable() //yaw pitch1 pitch2 失能
{
    //阻塞直到失能成功
    while (J4310_1.Err != 0) {
        DM_Disable(0x101);
        HAL_Delay(3);
    }
    while (J4310_2.Err != 0) {
        DM_Disable(0x102);
        HAL_Delay(3);
    }
    while (J4310_3.Err != 0) {
        DM_Disable(0x103);
        HAL_Delay(3);
    }
    Enable_flag = 0;
}

void Arm_Motor_Pos_cmd(uint8_t Posture) {
    for (uint8_t i = 0; i < 4; i++) {
        Pos[i] = Arm_Posture[Posture][i];
    }

    DM_SpeedPosition_cmd(&hcan1, 0x101, Vel[0], Pos[0]);
    HAL_Delay(5);
    DM_SpeedPosition_cmd(&hcan1, 0x102, Vel[1], Pos[1]);
    HAL_Delay(5);
    DM_SpeedPosition_cmd(&hcan1, 0x103, Vel[2], Pos[2]);
    HAL_Delay(5);

    //确保到位
    if (Posture != REMOTE_POSTURE) {
        while (!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
            (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
            (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
            DM_SpeedPosition_cmd(&hcan1, 0x101, Vel[0], Pos[0]);
            HAL_Delay(5);
            DM_SpeedPosition_cmd(&hcan1, 0x102, Vel[1], Pos[1]);
            HAL_Delay(5);
            DM_SpeedPosition_cmd(&hcan1, 0x103, Vel[2], Pos[2]);
            HAL_Delay(5);
        }
    }
}

/**
 * @brief 计算指定色温指定亮度的对应channel值
 * @param Temperature 输入色温， 3000-6500
 * @param Light 输入亮度 0-70
 */
void Arm_Light_cmd(uint16_t Temperature, uint8_t Light) {
    uint8_t channel1 = ((Temperature - 3000) / 3500.0) * Light;
    uint8_t channel2 = ((6500 - Temperature) / 3500.0) * Light;
    Light_Channel[0] = ((((channel1 > 148) ? 148 : channel1) < 0) ? 0 : (channel1 > 148) ? 148 : channel1);
    Light_Channel[1] = ((((channel2 > 148) ? 148 : channel2) < 0) ? 0 : (channel2 > 148) ? 148 : channel2);
    Light_cmd();
}

/**
 * @brief 机械臂关闭 包含缓慢关灯
 */
void Arm_Off() {
    Arm_Light_slow_OFF();
    for (uint8_t i = 0; i < 4; i++) { Arm_Light_cmd(Temperature, 0); }
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.7; }
    Vel[0] = 1.5;
    Arm_Motor_Pos_cmd(OFF_POSTURE);
    while (!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
        (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
        (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(OFF_POSTURE);
    }
    HAL_Delay(500);

    //关闭pitch3
    cmd_motor(0x200, 0, 0, 0, 0);
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_TIM_Base_Stop_IT(&htim2);
    cmd_motor(0x200, 0, 0, 0, 0);

    Arm_Motor_Disable();
    HAL_Delay(300);
}

void Arm_Qucik_Off() {
    for (uint8_t i = 0; i < 4; i++) { Arm_Light_cmd(Temperature, 0); }
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.7; }
    Vel[0] = 1.5;
    Arm_Motor_Pos_cmd(OFF_POSTURE);
    while (!((J4310_1.position - Pos[0] <= 0.1) && (J4310_1.position - Pos[0] >= -0.1) &&
        (J4310_2.position - Pos[1] <= 0.1) && (J4310_2.position - Pos[1] >= -0.1) &&
        (J4310_3.position - Pos[2] <= 0.1) && (J4310_3.position - Pos[2] >= -0.1))) {
        Arm_Motor_Pos_cmd(OFF_POSTURE);
    }
    HAL_Delay(500);

    //关闭pitch3
    cmd_motor(0x200, 0, 0, 0, 0);
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_TIM_Base_Stop_IT(&htim2);
    cmd_motor(0x200, 0, 0, 0, 0);

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
        for (uint8_t i = 0; i < 4; i++) { temp_Pos[i] = Arm_Posture[REMOTE_POSTURE][i]; }
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
        for (uint8_t i = 0; i < 4; i++) { Arm_Posture[REMOTE_POSTURE][i] = temp_Pos[i]; }
        Temperature = temp_Temperature;

        Arm_Motor_Pos_cmd(REMOTE_POSTURE);
        Arm_Light_cmd(Temperature, 244);
    }
    else {
        Vel[0] = 1.5;
        Vel[1] = 0.7;
        Vel[2] = 0.7;
        Vel[3] = 0.7;

        for (uint8_t i = 0; i < 4; i++) { Arm_Posture[REMOTE_POSTURE][i] = Arm_Posture[BASE_POSTURE][i]; }
    }
}

/**
 * @brief 缓启动灯光
 */
void Arm_Light_slow_ON() {
    float cnt = 0.0;
    for (int i = 0; i < 100; i++) {
        cnt = i * 0.015; // 计算当前 cnt 值
        Arm_Light_cmd(Temperature, 244 * sin(cnt));
    }
    Arm_Light_cmd(Temperature, 244); // 循环结束后设置为最大值
}

/**
 * @brief 缓关闭灯光
 */
void Arm_Light_slow_OFF() {
    float cnt = 0.0;
    for (int i = 40; i < 100; i++) {
        cnt = 3.1415927 / 2.0 + i * 0.015; // 计算当前 cnt 值
        Arm_Light_cmd(Temperature, 244 * sin(cnt));
    }
    Arm_Light_cmd(Temperature, 0); // 循环结束后设置为最大值
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

    Arm_Light_cmd(Temperature, Light);
}

void Arm_Remind_Sitting() {
    Arm_Motor_Pos_cmd(REMIND_SITTING_POSTURE);
    HAL_Delay(200);
    Pos[3] += 0.1;
    HAL_Delay(400);
    Pos[3] -= 0.1;
    HAL_Delay(400);
    Arm_Motor_Pos_cmd(BASE_POSTURE);
    HAL_Delay(8000);
}

void Arm_Looking_Forward() {
    Arm_Light_slow_OFF();
    Arm_Motor_Pos_cmd(REMIND_LOOKING_FORWARD_POSTURE);
    HAL_Delay(200);
    Pos[3] += 0.1;
    HAL_Delay(400);
    Pos[3] -= 0.1;
    HAL_Delay(400);
    Arm_Motor_Pos_cmd(BASE_POSTURE);
    Arm_Light_slow_ON();
    HAL_Delay(8000);
}

void Arm_Back() {
    Arm_Motor_Pos_cmd(BASE_POSTURE);
    for (uint8_t i = 0; i < 4; i++) { Pos[i] = Arm_Posture[BASE_POSTURE][i]; }
    for (uint8_t i = 0; i < 4; i++) { Arm_Posture[REMOTE_POSTURE][i] = Arm_Posture[BASE_POSTURE][i]; }
    for (uint8_t i = 1; i < 4; i++) { Vel[i] = 0.6; }
    Vel[0] = 1.5;
    Temperature = 6000;
    Arm_Light_cmd(Temperature, 244);
}

void Arm_Turn_Itself_Off() {
    Arm_Motor_Pos_cmd(BASE_POSTURE);
    for (uint8_t i = 0; i < 4; i++) { Vel[i] = 0.6; }
    HAL_Delay(200);
    //执行自动关机程序
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[1]);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[2]);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[3]);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[4]);
    HAL_Delay(600);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[5]);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[6]);
    HAL_Delay(500);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[7]);
    HAL_Delay(600);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[8]);
    HAL_Delay(600);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[9]);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[10]);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[11]);
    HAL_Delay(200);
    Arm_Motor_Pos_cmd(Turn_Itself_Off_Posture[12]);
    HAL_Delay(200);
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
    else if (RC.s1 == 3 && RC.s2 == 3) {
        Arm_Looking_Forward();
    }
    // else if (RC.s1 == 2 && RC.s2 == 2) {
    //     Arm_Turn_Itself_Off();
    // }
    else {
        Arm_Back();
    }
}
