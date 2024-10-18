#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/// 定义基础信息
#define ABS(x) ((x) > 0 ? (x) : -(x)) // 求绝对值
#define MIN(a, b) ((a) < (b) ? (a) : (b)) // 求最小值
#define MAX(a, b) ((a) > (b) ? (a) : (b)) // 求最大值
#define LIMIT(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))) // 限幅

/// 定义系统信息
#define TIMER_PRESCALER 6U  // 定时器预分频数
#define TIMER_PERIOD 1000U  // 定时器周期
#define FOC_CONTROL_FREQ 10000U	// FOC控制频率 = PWM频率(中央对齐模式,频率减半)(120MHz / 6 / 2 / 1000 = 10kHz)
#define FOC_CONTROL_PERIOD (1.0F / FOC_CONTROL_FREQ)	// FOC控制周期
#define ReductionRatio (-15)	//减速器减速比(负数代表减速器会反向)

// 定义编码器信息
#define ENCODER_PULSE 4095	// 编码器最大计数值
#define ANTICOGING_INCREMENT 4	// 抗齿槽力矩对应位置增量
#define ANTICOGING_TABLE_NUM (uint32_t)((ENCODER_PULSE + 1) / ANTICOGING_INCREMENT)	// 抗齿槽力矩表大小
#define Encoder_DT FOC_CONTROL_PERIOD // 编码器更新周期
#define PLL_BANDWIDTH (FOC_CONTROL_FREQ / 20)  // PLL带宽

// 定义检测信息
#define DETECTINGRESISTANCE_KI 20.0F // 检测电机电阻积分增益

typedef enum
{
    Warning_Null = 0,   // 空警告
    Warning_PowerLowVoltage,    // 电源电压过低
    Warning_TemperatureHigh,    // 温度过高
}WarningType; //警告类型

typedef enum
{
    Error_Null = 0,	// 空错误
    Error_Unknown,	// 未知错误
    Error_EncoderReadError, // 编码器读取错误
    Error_PowerLowVoltage,  // 电源电压过低
    Error_PowerHighVoltage, // 电源电压过高
    Error_PolePairsError,   // 极对数错误
    Error_ResistanceError,  // 电阻错误
    Error_InductanceError,  // 电感错误
    Error_DirectionError,   // 转向错误
    Error_MaxCurrentError,  // 最大电流错误

    /// @brief 电阻检测错误
    Error_DetectingResistance_Unknown = 100,	// 检测电机电阻-未知错误
    Error_DetectingResistance_OverVoltage,	// 检测电机电阻-过压
    Error_DetectingResistance_LargeCurrentError,	// 检测电机电阻-电流误差过大

    /// @brief 电感检测错误
    Error_DetectingInductance_Unknown = 200,	// 检测电机电感-未知错误

    /// @brief 编码器校准错误
    Error_EncoderOffsetCalibration_Unknown = 300,	// 编码器校准-未知错误
    Error_EncoderOffsetCalibration_OverTime,	// 编码器校准-超时
    Error_EncoderOffsetCalibration_EncoderError,	// 编码器校准-编码器错误
    Error_EncoderOffsetCalibration_PolePairsError,	// 编码器校准-极对数错误
}ErrorType; //错误类型

typedef struct
{
    uint8_t MotorID;	//电机ID
    uint8_t PolePairs;	//电机极对数
    int8_t Direction;	//电机转向
    float Resistance;	//电机电阻
    float Inductance;	//电机电感
    int32_t EncoderOffset;	//编码器偏置
    uint8_t MotorMode;	//电机模式
    uint8_t AnticoggingCalibratedFlag;	//抗齿槽力矩校准标志(1:已校准,0:未校准)
    // float DetectingCurrent;	//检测电流
    // float MaxDetectingVoltage;	//最大检测电压
    float MaxCurrent;	//最大电流
    float MaxSpeed;	//最大速度
    float SpeedKp;	//速度环P参数
    float SpeedKi;	//速度环I参数
    float PositionKp;	//位置环P参数
    float PositionKi;	//位置环I参数
}SystemConfigInfo;	//系统配置信息
extern SystemConfigInfo const* const SystemInfo;

#endif // !MAIN_H


