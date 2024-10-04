#include "Command.h"
#include "motor.h"

extern MotorInfo motor;
void SystemConfigInfoUpdate(void);

/// @brief 串口命令解析
/// @param data 要解析的命令
/// @param endFlag 命令结束标志
void UsartCommandAnalyze(char* data)
{
    unsigned short command = data[0] << 8 | data[1];
    switch (command)
    {
    case 'D' << 8 | 'R':  //检测电机电阻
        motor.mode = MM_DetectingResistance;
        break;
    case 'D' << 8 | 'I':  //检测电机电感
        motor.mode = MM_DetectingInductance;
        break;
    case 'D' << 8 | 'P':  //检测电机极对数
        motor.mode = MM_DetectingPolePairs;
        break;
    case 'D' << 8 | 'E':  //检测编码器
        motor.mode = MM_DetectingEncoder;
        break;
    case 'O' << 8 | 'L':  //开环控制
        motor.mode = MM_OpenLoop;
        break;
    case 'U' << 8 | 'P':  //更新系统配置信息
        SystemConfigInfoUpdate( );
        break;
    }
}




