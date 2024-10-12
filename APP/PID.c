#include "PID.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

/// @brief 位置式PID控制
/// @param info PID参数 
/// @param target 目标值
/// @param Value 实际值
void PIDSingleCalc(PIDInfo* info, const float target, const float Value)
{
    // 更新数据
    info->lastError = info->error;
    info->error = target - Value;
    // 计算微分
    info->output = (info->error - info->lastError) * info->Kd;
    // 计算比例
    info->output += info->error * info->Kp;
    // 抗积分饱和
    if (info->lastOutput > info->maxOutput) // (若lastOutput > MaxOutput ,则只累加负偏差)
        info->integral += info->error < 0 ? info->error : 0;
    else if (info->lastOutput < -info->maxOutput)   // (若lastOutput < -MaxOutput,则只累加正偏差)
        info->integral += info->error > 0 ? info->error : 0;
    else
        info->integral += info->error;
    LIMIT(info->integral, -info->maxIntegral, info->maxIntegral);   //积分限幅
    info->output += info->integral * info->Ki;  // 计算积分
    info->lastOutput = info->output;    //保存上次输出

    LIMIT(info->output, -info->maxOutput, info->maxOutput); //输出限幅
}

/// @brief 增量式PID控制
/// @param info PID参数
/// @param target 目标值
/// @param Value 实际值
void IncPIDSingleCalc(IncPIDInfo* info, const float target, const float Value)
{
    info->curError = target - Value;
    info->output = info->Kp * info->curError - info->Ki * info->lastError + info->Kd * info->llasError;
    info->llasError = info->lastError;
    info->lastError = info->curError;
    LIMIT(info->output, -info->maxOutput, info->maxOutput);
}


