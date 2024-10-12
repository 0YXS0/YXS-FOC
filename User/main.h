#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "PID.h"
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

/// 定义基础信息
#define ABS(x) ((x) > 0 ? (x) : -(x)) // 求绝对值
#define MIN(a, b) ((a) < (b) ? (a) : (b)) // 求最小值
#define MAX(a, b) ((a) > (b) ? (a) : (b)) // 求最大值
#define LIMIT(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))) // 限幅

/// 定义系统信息
#define ReductionRatio -15	//减速器减速比
#define FOC_CONTROL_FREQ 10000U	// FOC控制频率
#define FOC_CONTROL_PERIOD (1.0F / FOC_CONTROL_FREQ)	// FOC控制周期

// 定义编码器信息
#define ENCODER_PULSE 4095	// 编码器最大计数值
#define ANTICOGING_INCREMENT 4	// 抗齿槽力矩对应位置增量
#define ANTICOGING_TABLE_NUM (uint32_t)((ENCODER_PULSE + 1) / ANTICOGING_INCREMENT)	// 抗齿槽力矩表大小
#define Encoder_DT FOC_CONTROL_PERIOD // 编码器更新周期
#define PLL_BANDWIDTH (FOC_CONTROL_FREQ / 20)  // PLL带宽

// 定义检测信息
#define DETECTINGRESISTANCE_KI 12.0F // 检测电机电阻积分增益

#endif // !MAIN_H


