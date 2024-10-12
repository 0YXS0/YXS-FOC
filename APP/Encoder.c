#include "Encoder.h"
#include "math.h"
#include "Filter.h"
#include "main.h"

#define _PI 3.1415927F //PI
#define _2PI 6.2831853F //2PI
static const float PLL_Kp = 2.0F * PLL_BANDWIDTH; //PLL比例增益
static const float PLL_Ki = 0.25F * (PLL_Kp * PLL_Kp); //PLL积分增益
EncoderInfo Encoder = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //编码器信息
FirstOrderFilterInfo FilterInfo_EncoderSpeed = { 0.8F, 0.2F, 0 }; //编码器滤波器参数

// /// @brief 将值转换为-range~range范围内的值
// /// @param value 要转换的值
// /// @param range 范围
// /// @return 转换后的值
// static inline float wrap_pm(float value, float range)
// {
//     float t = fmodf(value + range, 2.0F * range);
//     if (t < 0) t += 2.0F * range;
//     return t - range;
// }

/// @brief 更新编码器值(角度、速度、累积角度)
/// @return 0:成功 <0:失败
int8_t Encoder_UpdateValue(void)
{
    int8_t ret = getEncoderRawCount((uint32_t*)&Encoder.RawCount); // 获取原始计数
    if (ret != 0) return ret; // 获取原始计数失败

    Encoder.DiffCount = Encoder.RawCount - Encoder.LastRawCount; // 计算计数差值
    if (Encoder.DiffCount > ENCODER_PULSE / 2) Encoder.DiffCount -= ENCODER_PULSE;  // 逆时针旋转溢出处理
    else if (Encoder.DiffCount < -ENCODER_PULSE / 2) Encoder.DiffCount += ENCODER_PULSE;    // 顺时针旋转溢出处理

    Encoder.AccCount += Encoder.DiffCount; // 累积计数
    Encoder.LastRawCount = Encoder.RawCount; // 更新上一次的原始计数

    // 预测计数值
    Encoder.EstimateAccCount += Encoder_DT * Encoder.EstimateSpeedCount; // 预测累积计数
    // Encoder.EstimateCount += Encoder_DT * Encoder.EstimateSpeedCount; // 预测单圈内计数
    // 离散相位检测
    float DeltaAccCount = (float)Encoder.AccCount - Encoder.EstimateAccCount; // 计算累积计数差值
    // float DeltaCount = (float)Encoder.RawCount - Encoder.EstimateCount; // 计算单圈内计数差值
    // DeltaCount = wrap_pm(DeltaCount, ENCODER_PULSE / 2); // 转换为-ENCODER_PULSE/2 ~ ENCODER_PULSE/2范围内的值
    // PLL反馈
    Encoder.EstimateAccCount += Encoder_DT * PLL_Kp * DeltaAccCount; // PLL比例增益
    // Encoder.EstimateCount += Encoder_DT * PLL_Kp * DeltaCount; // PLL比例增益
    // Encoder.EstimateCount = fmodf(Encoder.EstimateCount, ENCODER_PULSE);
    // if (Encoder.EstimateCount < 0) Encoder.EstimateCount += ENCODER_PULSE; // 转换为0~ENCODER_PULSE范围内的值
    Encoder.EstimateSpeedCount += Encoder_DT * PLL_Ki * DeltaAccCount; // PLL积分增益

    char snap_to_zero_Speed = 0;    // 速度向0对齐标志
    if (ABS(Encoder.EstimateSpeedCount) < 0.5F * Encoder_DT * PLL_Ki)
    {
        Encoder.EstimateSpeedCount = 0.0F; // 速度小于0.5*PLL_Ki时,置零
        snap_to_zero_Speed = 1;
    }

    // 插值计算   
    if (snap_to_zero_Speed) Encoder.IntterpolationValue = 0.5F;
    // else if (Encoder.DiffCount > 0) Encoder.IntterpolationValue = 0.0F;
    // else if (Encoder.DiffCount < 0) Encoder.IntterpolationValue = 1.0F;
    else
    {
        Encoder.IntterpolationValue += Encoder_DT * Encoder.EstimateSpeedCount;
        if (Encoder.IntterpolationValue > 1.0F) Encoder.IntterpolationValue = 1.0F;
        else if (Encoder.IntterpolationValue < 0.0F) Encoder.IntterpolationValue = 0.0F;
    }

    // 计算角度
    Encoder.Angle = fmodf((Encoder.RawCount + Encoder.IntterpolationValue - Encoder.OffsetCount) * _2PI / ENCODER_PULSE, _2PI); //计算角度
    if (Encoder.Angle < 0) Encoder.Angle += _2PI; // 转换为0~2PI范围内的值
    Encoder.AccAngle = Encoder.AccCount * _2PI / ENCODER_PULSE; // 计算累积角度
    Encoder.Speed = FirstOrderFilter(&FilterInfo_EncoderSpeed, Encoder.EstimateSpeedCount * _2PI / ENCODER_PULSE); // 计算速度
    return 0;
}