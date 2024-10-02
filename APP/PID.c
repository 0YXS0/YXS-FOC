#include "PID.h"

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

/// @brief PID控制
/// @param info PID参数 
/// @param target 目标值
/// @param Value 实际值
void PID_SingleCalc(PIDInfo* info, const float target, const float Value)
{
    //更新数据
    info->lastError = info->error;
    info->error = target - Value;
    //计算微分
    info->output = (info->error - info->lastError) * info->Kd;
    //计算比例
    info->output += info->error * info->Kp;
    //计算积分
    info->integral += info->error * info->Ki;
    LIMIT(info->integral, -info->maxIntegral, info->maxIntegral);//积分限幅
    info->output += info->integral;
    //输出限幅
    LIMIT(info->output, -info->maxOutput, info->maxOutput);
}


