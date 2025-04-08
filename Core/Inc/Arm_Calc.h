
#ifndef ARM_CALC_H
#define ARM_CALC_H

typedef struct Arm_Params_t
{
    double L1; // Length of link 1
    double L2; // Length of link 2
    double L3; // Length of link 3

    double theta2; // Joint angle 2
    double theta3; // Joint angle 3

    double q[3];
    double B_resume[2];
    double B[2];
    double target;
}Arm_Params_t;

#define PI 3.14159265

uint8_t Arm_Calculate(double target,double X_B,double Y_B,Arm_Params_t *arm_param);
double clamp(double value, double min, double max);

#endif //ARM_CALC_H
