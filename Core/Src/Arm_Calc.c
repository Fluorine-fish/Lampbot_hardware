#include "main.h"
#include <math.h>
#include "Arm_Calc.h"

#define PI 3.14159265

Arm_Params_t Arm_params = {160.0,179.5132307,45,0.418789,1.6580638556,{0,0,0},{0,0},{64.0,64.0},0.0};

double clamp(double value, double min, double max) {
    return (value < min) ? min : (value > max) ? max : value;
}

uint8_t Arm_Calculate(double target,double X_B,double Y_B,Arm_Params_t *arm_param) {
    arm_param->B[0] = X_B;
    arm_param->B[1] = Y_B;
    arm_param->target = target;
    arm_param->L1 = 160.0;
    arm_param->L2 = sqrt(73*73+164*164);
    arm_param->L3 = 45.0;
    arm_param->theta2 = atan(73.0/164.0);
    arm_param->theta3 = 1.6580638556;

    // 计算中间变量
    const double R = sqrt(arm_param->B[0]*arm_param->B[0] + arm_param->B[1]*arm_param->B[1]);

    // --------------- 检查分母和参数有效性 ---------------
    if (R == 0 || 2*arm_param->L1*R == 0 || 2*arm_param->L1*arm_param->L2 == 0) {
        return 1;
    }

    // 计算 q1 -------------------------------------------------
    double arg_q1 = (arm_param->L2*arm_param->L2 - R*R - arm_param->L1*arm_param->L1) / (2 * arm_param->L1 * R);
    if (fabs(arg_q1) > 1.0) {
        return 1;
    }
    double q1 = -atan2(Y_B, X_B) + acos(arg_q1);
    // q1 = fmod(q1 + 2*PI, 2*PI);    // 归一化到 [0, 2π)
    // if (q1 > PI) q1 -= 2*PI;       // 映射到 [-π, π]
    q1 = clamp(q1, 0.0, PI);       // 限制在 [0, π]

    // 计算 q2 -------------------------------------------------
    double arg_q2 = (arm_param->L1 * arm_param->L1 + arm_param->L2 * arm_param->L2 - R*R) / (2 * arm_param->L1 * arm_param->L2);
    if (fabs(arg_q2) > 1.0) {
        return 2;
    }
    double q2 = acos(arg_q2) - arm_param->theta2;
    q2 = clamp(q2, 0.0, PI);       // 限制在 [0, π]

    // 计算 q3 -------------------------------------------------
    arm_param->q[2] = target - arm_param->q[1] - arm_param->theta2 + arm_param->q[0] + arm_param->theta3;
    arm_param->q[2] = clamp(arm_param->q[2], -1.66, 1.66);   // 限制在 [-1.66, 1.66]

    // //运行正运动学求解进行比较判断逆解正确性
    // arm_param->B_resume[0] = arm_param->L1*cos(PI-arm_param->q[0])+arm_param->L2*cos(arm_param->q[2]+arm_param->theta2-arm_param->q[1]);
    // arm_param->B_resume[1] = arm_param->L1*sin(PI-arm_param->q[0])+arm_param->L2*sin(arm_param->q[2]+arm_param->theta2-arm_param->q[1]);
    // // if(fabs(arm_param->B_resume[0]-arm_param->B[0]) > 0.01 || fabs(arm_param->B_resume[1]-arm_param->B[1]) > 0.01)
    // // {
    // //     printf("错误：逆解误差过大");
    // //     return 3;
    // // }

    // 输出结果
    // printf("q1 = %.4f rad (%.2f°)\n", arm_param->q[0], arm_param->q[0]*180/PI);
    // printf("q2 = %.4f rad (%.2f°)\n", arm_param->q[1], arm_param->q[1]*180/PI);
    // printf("q3 = %.4f rad (%.2f°)\n", arm_param->q[2], arm_param->q[2]*180/PI);
    return 0;
}
