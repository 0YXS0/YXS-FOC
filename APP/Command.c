#include "Command.h"
#include "motor.h"
#include "stdlib.h"

extern MotorInfo motor;   //电机信息
extern uint8_t PrintfConfigInfoFlag;    //打印配置信息标志位
extern uint8_t PrintfDebugInfoFlag;    //打印调试信息标志位
void SystemConfigInfoUpdate(void);

/// @brief 串口命令解析
/// @param data 要解析的命令
/// @param endFlag 命令结束标志
void UsartCommandAnalyze(char* data)
{
    unsigned short command = data[0] << 8 | data[1];
    switch (command)
    {
    case 'N' << 8 | 'U':  // 停止电机
        motor.mode = MM_NULL;
        break;
    case 'D' << 8 | 'R':  // 检测电机电阻
        motor.mode = MM_DetectingResistance;
        break;
    case 'D' << 8 | 'L':  // 检测电机电感
        motor.mode = MM_DetectingInductance;
        break;
    case 'C' << 8 | 'E':  // 检测编码器
        motor.mode = MM_EncoderCalibration;
        break;
    case 'A' << 8 | 'C':  // 抗齿槽力矩校准
        motor.mode = MM_AnticoggingCalibration;
        break;
    case 'O' << 8 | 'L':  // 开环控制
        motor.mode = MM_OpenLoop;
        break;
    case 'U' << 8 | 'P':  // 更新系统配置信息
        SystemConfigInfoUpdate( );
        break;
    case 'S' << 8 | 'C':  // 打印配置信息
        PrintfConfigInfoFlag = 1;
        break;
    case 'D' << 8 | 'I':  // 打印调试信息
        PrintfDebugInfoFlag = atoi(data + 2);
        break;
    case 'S' << 8 | 'M': // 设置电机模式
        motor.mode = (MotorMode)(data[2] - '0');
        break;
    case 'C' << 8 | 'P':  // 设置电流环Kp
        motor.PIDInfoIQ.Kp = strtof(data + 2, NULL);
        break;
    case 'C' << 8 | 'I':  // 设置电流环Ki
        motor.PIDInfoIQ.Ki = strtof(data + 2, NULL);
        break;
    case 'C' << 8 | 'T': // 设置目标电流
        motor.TargetCurrent = strtof(data + 2, NULL);
        break;
    case 'S' << 8 | 'P':  // 设置速度环Kp
        motor.PIDInfoSpeed.Kp = strtof(data + 2, NULL);
        break;
    case 'S' << 8 | 'I':  // 设置速度环Ki
        motor.PIDInfoSpeed.Ki = strtof(data + 2, NULL);
        break;
    case 'A' << 8 | 'F':  // 设置速度环Ki
        motor.AnticoggingCalibratedFlag = atoi(data + 2);
        break;
        // case 'S' << 8 | 'D':  // 设置速度环Kd
        //     motor.PIDInfoSpeed.Kd = strtof(data + 2, NULL);
        break;
    case 'S' << 8 | 'T': // 设置目标速度
        motor.TargetSpeed = strtof(data + 2, NULL);
        break;
    case 'P' << 8 | 'P':  // 设置位置环Kp
        motor.PIDInfoPosition.Kp = strtof(data + 2, NULL);
        break;
    case 'P' << 8 | 'I':  // 设置位置环Ki
        motor.PIDInfoPosition.Ki = strtof(data + 2, NULL);
        break;
    case 'P' << 8 | 'D':  // 设置位置环Kd
        motor.PIDInfoPosition.Kd = strtof(data + 2, NULL);
        break;
    case 'P' << 8 | 'T': // 设置目标位置
        motor.TargetPosition = strtof(data + 2, NULL);
        break;
    case 'M' << 8 | 'C': // 设置电机最大电流
        motor.MaxCurrent = strtof(data + 2, NULL);
        break;
    case 'M' << 8 | 'S': // 设置电机最大速度
        motor.MaxSpeed = strtof(data + 2, NULL);
        break;
    }
}





