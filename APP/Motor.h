#ifndef MOTOR_H
#define MOTOR_H

#include "gd32f30x.h"

typedef struct
{
    uint8_t MotorID;    //电机ID
    uint8_t PolePairs;  //电机极对数
    uint32_t MAXPulse;  //PWM定时器最大计数值
    float Udc;  //电机控制电源电压
    float Angle;    //电机角度
    float Ia;   //U相电流
    float Ib;   //V相电流
    float Ic;   //W相电流
    float Ialpha;   //alpha轴
    float Ibeta;    //beta轴
    float Id;   //d轴
    float Iq;   //q轴

    float Ualpha;   //alpha轴
    float Ubeta;    //beta轴
    float Uq;   //q轴
    float Ud;   //d轴
    float Ua;   //U相电压
    float Ub;   //V相电压
    float Uc;   //W相电压

    uint32_t PulseA;    //U相PWM占空比
    uint32_t PulseB;    //V相PWM占空比
    uint32_t PulseC;    //W相PWM占空比
}MotorInfo;

void Clarke_Transf(MotorInfo* info);    //Clarke变换
void Rev_Clarke_Transf(MotorInfo* info);    //逆Clarke变换  
void Park_Transf(MotorInfo* info);  //Park变换  
void Rev_Park_Transf(MotorInfo* info);  //逆Park变换
void SVPWM(MotorInfo* info);    //SVPWM
float DetectingResistance(float DetectingCurrent, float MaxDetectingVoltage);    //检测电机电阻

#endif // !MOTOR_H




