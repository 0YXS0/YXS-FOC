#ifndef MAIN_H
#define MAIN_H

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
#define ENCODER_PULSE 4096	// 编码器最大计数值
#define Encoder_DT FOC_CONTROL_PERIOD // 编码器更新周期
#define PLL_BANDWIDTH 1000  // PLL带宽

// 定义检测信息
#define DETECTINGRESISTANCE_KI 12.0F // 检测电机电阻积分增益

#endif // !MAIN_H


