#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "main.h"
#include "PID.h"
typedef enum
{
    MM_NULL = 0,	//空模式
    MM_DetectingResistance,	//检测电机电阻
    MM_DetectingInductance,	//检测电机电感
    MM_EncoderCalibration,	//检测编码器
    MM_CurrentControl,	//电流控制
    MM_SpeedControl,	//速度控制
    MM_PositionControl,	//位置控制
    MM_OpenLoop,	//开环控制
    MM_AnticoggingCalibration,	//抗齿槽力矩校准
    MM_Error	//错误
}MotorMode;	//电机模式
typedef struct
{
    MotorMode mode;    //电机控制模式
    uint8_t MotorID;    //电机ID
    uint8_t PolePairs;  //电机极对数
    uint32_t MAXPulse;  //PWM定时器最大计数值
    int8_t Direction;  //电机转向
    float Resistance;   //电机电阻
    float Inductance;   //电机电感
    // float DetectingCurrent; //检测电流
    // float MaxDetectingVoltage;  //最大检测电压
    float MaxCurrent;   //最大电流
    float MaxSpeed; //最大速度
    float Udc;  //母线电压
    float Angle;    //电机电角度
    float TargetCurrent;    //目标电流
    float TargetSpeed;  //目标速度
    float TargetPosition;   //目标位置
    uint8_t AnticoggingCalibratedFlag;    //抗齿槽力矩校准标志(0:已校准,!0:未校准)
    float AnticogongTorqueTable[ANTICOGING_TABLE_NUM]; // 抗齿槽力矩表
    WarningType WarningInfo;    //警告信息
    ErrorType ErrorInfo;    //错误信息

    float Ia;   //U相电流
    float Ib;   //V相电流
    float Ic;   //W相电流
    float Ialpha;   //alpha轴
    float Ibeta;    //beta轴
    float Id;   //d轴
    float Iq;   //q轴

    float sinValue; //角度sin值
    float cosValue; //角度cos值
    PIDInfo PIDInfoIQ; // q轴电流环PID
    PIDInfo PIDInfoID; // d轴电流环PID
    PIDInfo PIDInfoSpeed;   //速度环PID
    PIDInfo PIDInfoPosition;    //位置环PID

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
void OpenPWM(void); //开启PWM
void ClosePWM(void);    //关闭PWM
void Clarke_Transf(MotorInfo* info);    //Clarke变换
void Rev_Clarke_Transf(MotorInfo* info);    //逆Clarke变换  
void Park_Transf(MotorInfo* info);  //Park变换  
void Rev_Park_Transf(MotorInfo* info);  //逆Park变换
void SVPWM(MotorInfo* info);    //SVPWM
void ApplyMotorInfo(MotorInfo* info);  //将电机信息应用到电机
int8_t DetectingResistance(MotorInfo* info, float DetectingCurrent, float MaxDetectingVoltage);    //检测电机电阻
int8_t DetectingInductance(MotorInfo* info, float DetectingVoltage);    //检测电机电感
int8_t EncoderOffsetCalibration(MotorInfo* info);    //编码器校准
int8_t AnticoggingCalibration(MotorInfo* info);    //抗齿槽力矩校准
void CheckMotorInfoVality(MotorInfo* info, uint8_t PrintfFlag);   //检测电机信息有效性并更新PID参数

#endif // !MOTOR_H




