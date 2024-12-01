#include "Command.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "main.h"
#include "MMPrintf.h"
#include "Delay.h"
#include "motor.h"
#include "Encoder.h"
#include "CAN.h"

extern MotorInfo motor; // 电机信息
extern EncoderInfo Encoder; // 编码器信息
extern uint8_t PrintfConfigInfoFlag;    // 打印配置信息标志位
extern uint8_t PrintfDebugInfoFlag; // 打印调试信息标志位
extern uint8_t HeartbeatFlag;  // 心跳是否开启标志
extern uint8_t HeartbeatCycle; // 心跳周期
extern void SystemConfigInfoSave(void);
extern void InterfaceModeSelect(int8_t mode);

/// @brief 串口命令解析
/// @param data 要解析的命令
/// @param endFlag 命令结束标志
void UsartCommandAnalyze(char* data)
{
    unsigned short command = data[0] << 8 | data[1];
    switch (command)
    {
    case 'R' << 8 | 'S':  // 重启系统
        ClosePWM( );    // 关闭PWM
        __disable_irq( );   // 关闭中断
        NVIC_SystemReset( );    // 系统重启
        break;
    case 'I' << 8 | 'D': // 设置电机ID
        motor.MotorID = (uint8_t)atoi(data + 2) & 0x3F;
        break;
    case 'N' << 8 | 'U':  // 停止电机
        motor.NextMode = MM_NULL;
        break;
    case 'D' << 8 | 'R':  // 检测电机电阻
        motor.NextMode = MM_DetectingResistance;
        break;
    case 'D' << 8 | 'L':  // 检测电机电感
        motor.NextMode = MM_DetectingInductance;
        break;
    case 'C' << 8 | 'E':  // 检测编码器
        motor.NextMode = MM_EncoderCalibration;
        break;
    case 'A' << 8 | 'C':  // 抗齿槽力矩校准
        motor.NextMode = MM_AnticoggingCalibration;
        break;
    case 'U' << 8 | 'P':  // 更新系统配置信息
        SystemConfigInfoSave( );
        break;
    case 'S' << 8 | 'C':  // 打印配置信息
        PrintfConfigInfoFlag = 1;
        break;
    case 'D' << 8 | 'I':  // 打印调试信息
        PrintfDebugInfoFlag = atoi(data + 2);
        break;
    case 'S' << 8 | 'M': // 设置电机模式
        motor.NextMode = (MotorMode)(data[2] - '0');
        break;
    case 'C' << 8 | 'P':  // 设置电流环Kp
        motor.PIDInfoIQ.Kp = strtof(data + 2, NULL);
        UpdatePIDInfo(&motor);  // 更新PID参数
        break;
    case 'C' << 8 | 'I':  // 设置电流环Ki
        motor.PIDInfoIQ.Ki = strtof(data + 2, NULL);
        UpdatePIDInfo(&motor);  // 更新PID参数
        break;
    case 'C' << 8 | 'T': // 设置目标电流
        motor.TargetCurrent = strtof(data + 2, NULL);
        break;
    case 'S' << 8 | 'P':  // 设置速度环Kp
        motor.PIDInfoSpeed.Kp = strtof(data + 2, NULL);
        UpdatePIDInfo(&motor);  // 更新PID参数
        break;
    case 'S' << 8 | 'I':  // 设置速度环Ki
        motor.PIDInfoSpeed.Ki = strtof(data + 2, NULL);
        UpdatePIDInfo(&motor);  // 更新PID参数
        break;
    case 'S' << 8 | 'D':  // 设置速度环Ki
        motor.PIDInfoSpeed.Kd = strtof(data + 2, NULL);
        break;
    case 'S' << 8 | 'T': // 设置目标速度
        motor.AxisTargetSpeed = strtof(data + 2, NULL);
        break;
    case 'P' << 8 | 'P':  // 设置位置环Kp
        motor.PIDInfoPosition.Kp = strtof(data + 2, NULL);
        UpdatePIDInfo(&motor);  // 更新PID参数
        break;
    case 'P' << 8 | 'I':  // 设置位置环Ki
        motor.PIDInfoPosition.Ki = strtof(data + 2, NULL);
        UpdatePIDInfo(&motor);  // 更新PID参数
        break;
    case 'P' << 8 | 'D':  // 设置位置环Ki
        motor.PIDInfoPosition.Kd = strtof(data + 2, NULL);
        break;
    case 'P' << 8 | 'T': // 设置目标位置
        motor.AxisTargetPosition = strtof(data + 2, NULL);
        break;
    case 'L' << 8 | 'C': // 设置电机限制电流
        motor.LimitCurrent = strtof(data + 2, NULL);
        break;
    case 'L' << 8 | 'S': // 设置电机限制速度
        motor.AxisLimitSpeed = strtof(data + 2, NULL);
        break;
    case 'D' << 8 | 'S': // 设置板载4P接口为烧录模式
        ClosePWM( );    // 关闭PWM
        __disable_irq( );   // 关闭中断
        InterfaceModeSelect(0);
        break;
    default:
        MM_printf("Unknown command:%c%c\n", data[0], data[1]);
        break;
    }
}

/// @brief can发送心跳帧
void canSendHeartbeatFrame(void)
{
    static can_trasnmit_message_struct m;
    m.tx_sfid = motor.MotorID << 5 | CanCommand_Hearbeat; // 标准帧ID
    m.tx_ff = CAN_FF_STANDARD;  // 标准帧
    m.tx_ft = CAN_FT_DATA;  // 数据帧
    m.tx_dlen = 7;  // 数据长度
    m.tx_data[0] = motor.CurMode;   // 当前模式
    switch (motor.CurMode)
    {
    case MM_CurrentControl:
        memcpy(m.tx_data + 1, &motor.Iq, 4);
        break;
    case MM_SpeedControl:
        memcpy(m.tx_data + 1, &motor.AxisSpeed, 4);
        break;
    case MM_PositionControl:
        memcpy(m.tx_data + 1, &motor.AxisAccPosition, 4);
        break;
    default:
        memset(m.tx_data + 1, 0x00, 4);
        break;
    }
    m.tx_data[5] = motor.WarningInfo;   // 警告信息
    m.tx_data[6] = motor.ErrorInfo; // 错误信息
    canSend(CAN0, &m);
}

/// @brief 依据接收到的数据索引帧拼接数据并发送
/// @param msg 接收到的索引数据帧
static void canReturnDataConcatenation(can_receive_message_struct* msg)
{
    int count = 0;
    static can_trasnmit_message_struct m;
    for (uint8_t i = 0; i < msg->rx_dlen; i++)
    {
        switch (msg->rx_data[i])
        {
        case DataIndex_MotorID:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.MotorID;
            break;
        case DataIndex_CurMode:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.CurMode;
            break;
        case DataIndex_PolePairs:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.PolePairs;
            break;
        case DataIndex_Resistance:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.Resistance, 4);
            count += 4;
            break;
        case DataIndex_Inductance:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.Inductance, 4);
            count += 4;
            break;
        case DataIndex_Udc:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.Udc, 4);
            count += 4;
            break;
        case DataIndex_Temp:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.Temp, 4);
            count += 4;
            break;
        case DataIndex_Current:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.Iq, 4);
            count += 4;
            break;
        case DataIndex_TargetCurrent:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.TargetCurrent, 4);
            count += 4;
            break;
        case DataIndex_LimitCurrent:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.LimitCurrent, 4);
            count += 4;
            break;
        case DataIndex_Speed:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.AxisSpeed, 4);
            count += 4;
            break;
        case DataIndex_TargetSpeed:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.AxisTargetSpeed, 4);
            count += 4;
            break;
        case DataIndex_LimitSpeed:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.AxisLimitSpeed, 4);
            count += 4;
            break;
        case DataIndex_Position:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.AxisAccPosition, 4);
            count += 4;
            break;
        case DataIndex_TargetPosition:
            if (count > 4) goto end;
            memcpy(m.tx_data + count, &motor.AxisTargetPosition, 4);
            count += 4;
            break;
        case DataIndex_IsOpenAntiCoggingFlag:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.IsOpenAntiCoggingFlag;
            break;
        case DataIndex_AnticoggingCalibratedFlag:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.AnticoggingCalibratedFlag;
            break;
        case DataIndex_WarningInfo:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.WarningInfo;
            break;
        case DataIndex_ErrorInfo:
            if (count > 7) goto end;
            m.tx_data[count++] = motor.ErrorInfo;
            break;
        case DataIndex_HeartbeatCycle:
            if (count > 7) goto end;
            m.tx_data[count++] = HeartbeatCycle;
            break;
        default:
            goto end;
        }
    }
end:
    m.tx_sfid = motor.MotorID << 5 | CanCommand_returnData; // 标准帧ID
    m.tx_ff = CAN_FF_STANDARD;  // 标准帧
    m.tx_ft = CAN_FT_DATA;  // 数据帧
    m.tx_dlen = count;  // 数据长度
    canSend(CAN0, &m);
}


/// @brief can命令解析
/// @param msg 接收到的数据帧
void CanCommandAnalyze(can_receive_message_struct* msg)
{
    switch (msg->rx_sfid & 0x0000001F)
    {
    case CanCommand_setMotorMode:    // 设置电机模式
        motor.NextMode = (MotorMode)msg->rx_data[0];
        break;
    case CanCommand_setTargetPosition:   // 设置目标位置
        motor.AxisTargetPosition = *(float*)(msg->rx_data);
        break;
    case CanCommand_setTargetSpeed:  // 设置目标速度
        motor.AxisTargetSpeed = *(float*)(msg->rx_data);
        break;
    case CanCommand_setTargetCurrent:    // 设置目标电流(力矩)
        motor.TargetCurrent = *(float*)(msg->rx_data);
        break;
    case CanCommand_setLimitSpeedANDCurrent: // 设置速度和电流限制
        motor.AxisLimitSpeed = *(float*)(msg->rx_data);
        motor.LimitCurrent = *(float*)(msg->rx_data + 4);
        break;
    case CanCommand_SaveConfig:  // 保存配置
        SystemConfigInfoSave( );
        break;
    case CanCommand_setHeartbeat:   // 设置心跳周期
        HeartbeatFlag = msg->rx_data[0];
        HeartbeatCycle = msg->rx_data[1];
        break;
    case CanCommand_setAntiCogging: // 开启抗齿槽力矩补偿
        motor.IsOpenAntiCoggingFlag = msg->rx_data[0];
        break;
    case CanCommand_readData:   // 读取数据
        canReturnDataConcatenation(msg);
        break;
    case CanCommand_InterfaceModeSelect: // 选择板载4P接口的模式
        InterfaceModeSelect(-1);
        break;
    default:
        break;
    }
}

