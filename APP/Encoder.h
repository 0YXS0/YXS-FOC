#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
typedef enum
{
    EVT_Angle = 1,   //原始角度
    EVT_AccAngle,   //累积角度
    EVT_Speed,      //速度
    EVT_Count,      //原始计数
    EVT_AccCount,   //累积计数
}EncoderValueType;  //编码器值类型

typedef struct
{
    int32_t RawCount; //原始计数
    int32_t AccCount; //累积计数
    int32_t LastRawCount; //上一次的原始计数
    int32_t DiffCount; //计数差值

    float EstimateAccCount; //预测累积计数    
    float EstimateCount; //预测单圈内计数
    float EstimateSpeedCount; //预测速度计数

    int32_t OffsetCount; //偏置计数
    float IntterpolationValue; //插值值

    float Angle; //原始角度
    float AccAngle; //累积角度
    float Speed;    //速度
} EncoderInfo;  //编码器信息
extern EncoderInfo Encoder;  //编码器信息

float Encoder_GetValue(EncoderValueType type);  //获取编码器值
void Encoder_UpdateValue(void);  //更新编码器值

#endif // !ENCODER_H


