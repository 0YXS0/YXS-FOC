#ifndef COMMAND_H
#define COMMAND_H

#include "gd32f30x_can.h"

typedef enum __DataIndex
{
    DataIndex_MotorID = 0,
    DataIndex_CurMode,
    DataIndex_PolePairs,
    DataIndex_Resistance,
    DataIndex_Inductance,
    DataIndex_MaxCurrent,
    DataIndex_MaxSpeed,
    DataIndex_Udc,
    DataIndex_Temp,
    DataIndex_TargetCurrent,
    DataIndex_LimitCurrent,
    DataIndex_TargetSpeed,
    DataIndex_LimitSpeed,
    DataIndex_TargetPosition,
    DataIndex_IsOpenAntiCoggingFlag,
    DataIndex_AnticoggingCalibratedFlag,
    DataIndex_WarningInfo,
    DataIndex_ErrorInfo,
    DataIndex_HeartbeatCycle,
}DataIndex;

typedef enum __CanCommand
{
    CanCommand_Hearbeat = 0x00, // 心跳
    CanCommand_OpenMotor,   // 开启电机
    CanCommand_CloseMotor,  // 关闭电机
    CanCommand_setMotorMode,    // 设置电机模式
    CanCommand_setTargetPosition,   // 设置目标位置
    CanCommand_setTargetSpeed,  // 设置目标速度
    CanCommand_setTargetCurrent,    // 设置目标电流(力矩)
    CanCommand_setLimitSpeedANDCurrent, // 设置速度和电流限制
    CanCommand_SaveConfig,  // 保存配置
    CanCommand_setHeartbeat,   // 设置心跳周期
    CanCommand_setAntiCogging, // 开启抗齿槽力矩补偿

    CanCommand_readData = 0x1E, // 读取数据
    CanCommand_returnData = 0x1F,   // 返回数据
}CanCommand;

void UsartCommandAnalyze(char* data);   // 串口命令解析
void canSendHeartbeatFrame(void);   // 发送心跳帧
void CanCommandAnalyze(can_receive_message_struct* msg);    // can命令解析

#endif // !COMMAND_H
