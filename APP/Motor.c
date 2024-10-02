#include "Motor.h"
#include "fast_sin.h"
#include "math.h"

/// @brief Clarke变换
/// @param info 电机信息
void Clarke_Transf(MotorInfo* info)
{
    info->Ialpha = (2 * info->Ia - info->Ib - info->Ic) * 0.33333333f;
    info->Ibeta = (info->Ib - info->Ic) * 0.57735026F;   //0.57735026 = sqrt(3)/3
}

void Rev_Clarke_Transf(MotorInfo* info)
{
    info->Ua = info->Ualpha;
    info->Ub = -0.5f * info->Ualpha + 0.8660254f * info->Ubeta;  //0.8660254 = sqrt(3)/2
    info->Uc = -0.5f * info->Ualpha - 0.8660254f * info->Ubeta;
}

/// @brief Park变换
/// @param info 电机信息
void Park_Transf(MotorInfo* info)
{
    // float cosValue = arm_cos_f32(info->Angle);
    // float sinValue = arm_sin_f32(info->Angle);
    float sinValue, cosValue;
    fast_sin_cos(info->Angle, &sinValue, &cosValue);
    info->Id = info->Ialpha * cosValue + info->Ibeta * sinValue;
    info->Iq = -info->Ialpha * sinValue + info->Ibeta * cosValue;
}

/// @brief 逆Park变换
/// @param info 电机信息
void Rev_Park_Transf(MotorInfo* info)
{
    // float cosValue = arm_cos_f32(info->Angle);
    // float sinValue = arm_sin_f32(info->Angle);
    float sinValue, cosValue;
    fast_sin_cos(info->Angle, &sinValue, &cosValue);
    info->Ualpha = info->Ud * cosValue - info->Uq * sinValue;
    info->Ubeta = info->Ud * sinValue + info->Uq * cosValue;
}

void SVPWM(MotorInfo* info)
{
    uint8_t sector = 0; //扇区
    float temp = 0, Tx = 0, Ty = 0, Tz = 0;

    //判断所处扇区
    temp = 1.73205078F * info->Ualpha;
    sector = info->Ubeta >= 0.0F ? 1 : 0;
    sector = temp >= info->Ubeta ? sector | 0x02 : sector;
    sector = -temp >= info->Ubeta ? sector | 0x04 : sector;

    //计算时间比例
    switch (sector)
    {
    case 3: //第一扇区
        Tx = (1.5F * info->Ualpha - 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = 1.73205078F * info->Ubeta * info->MAXPulse / info->Udc;
        if (Tx + Ty > info->MAXPulse)
        {
            Tx = info->MAXPulse * Tx / (Tx + Ty);
            Ty = info->MAXPulse * Ty / (Tx + Ty);
        }
        Tz = (info->MAXPulse - Tx - Ty) / 2;
        info->PulseA = Tz;
        info->PulseB = Tx + Tz;
        info->PulseC = Ty + info->PulseB;
        break;
    case 1: //第二扇区
        Tx = (1.5F * info->Ualpha + 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = (-1.5F * info->Ualpha + 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        if (Tx + Ty > info->MAXPulse)
        {
            Tx = info->MAXPulse * Tx / (Tx + Ty);
            Ty = info->MAXPulse * Ty / (Tx + Ty);
        }
        Tz = (info->MAXPulse - Tx - Ty) / 2;
        info->PulseA = Ty + Tz;
        info->PulseB = Tz;
        info->PulseC = Tx + info->PulseA;
        break;
    case 5: //第三扇区
        Tx = 1.73205078F * info->Ubeta * info->MAXPulse / info->Udc;
        Ty = (-1.5F * info->Ualpha - 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        if (Tx + Ty > info->MAXPulse)
        {
            Tx = info->MAXPulse * Tx / (Tx + Ty);
            Ty = info->MAXPulse * Ty / (Tx + Ty);
        }
        Tz = (info->MAXPulse - Tx - Ty) / 2;
        info->PulseA = Ty + Tx + Tz;
        info->PulseB = Tz;
        info->PulseC = Tx + Tz;
        break;
    case 2: //第四扇区
        Tx = (-1.5F * info->Ualpha + 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = -1.73205078F * info->Ubeta * info->MAXPulse / info->Udc;
        if (Tx + Ty > info->MAXPulse)
        {
            Tx = info->MAXPulse * Tx / (Tx + Ty);
            Ty = info->MAXPulse * Ty / (Tx + Ty);
        }
        Tz = (info->MAXPulse - Tx - Ty) / 2;
        info->PulseA = Tx + Ty + Tz;
        info->PulseB = Ty + Tz;
        info->PulseC = Tz;
        break;
    case 6: //第五扇区
        Tx = (-1.5F * info->Ualpha - 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = (1.5F * info->Ualpha - 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        if (Tx + Ty > info->MAXPulse)
        {
            Tx = info->MAXPulse * Tx / (Tx + Ty);
            Ty = info->MAXPulse * Ty / (Tx + Ty);
        }
        Tz = (info->MAXPulse - Tx - Ty) / 2;
        info->PulseA = Tx + Tz;
        info->PulseB = Ty + info->PulseA;
        info->PulseC = Tz;
        break;
    case 4: //第六扇区
        Tx = -1.73205078F * info->Ubeta * info->MAXPulse / info->Udc;
        Ty = (1.5F * info->Ualpha + 0.866025388F * info->Ubeta) * info->MAXPulse / info->Udc;
        if (Tx + Ty > info->MAXPulse)
        {
            Tx = info->MAXPulse * Tx / (Tx + Ty);
            Ty = info->MAXPulse * Ty / (Tx + Ty);
        }
        Tz = (info->MAXPulse - Tx - Ty) / 2;
        info->PulseA = Tz;
        info->PulseB = Tx + Ty + Tz;
        info->PulseC = Ty + Tz;
        break;
    }
}

/// @brief 检测电机电阻
/// @param DetectingCurrent 检测电流(A)
/// @param MaxDetectingVoltage 最大检测电压(V)
/// @return 电机电阻(欧姆)
float DetectingResistance(float DetectingCurrent, float MaxDetectingVoltage)
{
    float Resistance = 0;
    float Voltage = 0;
    Voltage = MaxDetectingVoltage * DetectingCurrent / 3.3F;
    Resistance = Voltage / DetectingCurrent;
    return Resistance;
}