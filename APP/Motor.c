#include "Motor.h"
#include "fast_sin.h"
#include "math.h"
#include "main.h"

#define SQRT3 1.73205078F   // sqrt(3)
#define SQRT3_DIV_2 0.866025388F    // sqrt(3)/2
#define _1_DIV_3 0.33333333F    // 1/3
#define _2_DIV_3 0.66666666F    // 2/3 

unsigned char PWMState = 0; //PWM状态(0:关闭,1:开启)
void OpenPWM(void)
{
    if (PWMState == 1) return;
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0U); // U相
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0U); // V相
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0U); // W相

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
    PWMState = 1;
}

void ClosePWM(void)
{
    if (PWMState == 0) return;
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0U); // U相
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0U); // V相
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0U); // W相

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);
    PWMState = 0;
}

/// @brief 设置PWM
/// @param motor 电机信息
static inline void setPWM(MotorInfo* motor)	//设置PWM
{
    if (motor->Direction)
    {
        // 设置占空比
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, motor->PulseA); //U相
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, motor->PulseB); //V相
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, motor->PulseC); //W相
    }
    else
    {
        // 设置占空比
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, motor->PulseA); //U相
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, motor->PulseC); //V相
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, motor->PulseB); //W相
    }
    // ADC采样控制PWM
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, MIN(MAX(MAX(motor->PulseA, motor->PulseB), motor->PulseC) + 75, 990));
    // timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, motor->MAXPulse - 10);
}

/// @brief Clarke变换
/// @param info 电机信息
void Clarke_Transf(MotorInfo* info)
{
    info->Ialpha = (2 * info->Ia - info->Ib - info->Ic) * _1_DIV_3;
    info->Ibeta = (info->Ib - info->Ic) * 0.57735026F;   //0.57735026 = sqrt(3)/3
}

void Rev_Clarke_Transf(MotorInfo* info)
{
    info->Ua = info->Ualpha;
    info->Ub = -0.5f * info->Ualpha + SQRT3_DIV_2 * info->Ubeta;  //0.8660254 = sqrt(3)/2
    info->Uc = -0.5f * info->Ualpha - SQRT3_DIV_2 * info->Ubeta;
}

/// @brief Park变换
/// @param info 电机信息
void Park_Transf(MotorInfo* info)
{
    info->Id = info->Ialpha * info->cosValue + info->Ibeta * info->sinValue;
    info->Iq = -info->Ialpha * info->sinValue + info->Ibeta * info->cosValue;
}

/// @brief 逆Park变换
/// @param info 电机信息
void Rev_Park_Transf(MotorInfo* info)
{
    info->Ualpha = info->Ud * info->cosValue - info->Uq * info->sinValue;
    info->Ubeta = info->Ud * info->sinValue + info->Uq * info->cosValue;
}

void SVPWM(MotorInfo* info)
{
    uint8_t sector = 0; //扇区
    float temp = 0, Tx = 0, Ty = 0, Tz = 0;

    //判断所处扇区
    temp = SQRT3 * info->Ualpha;
    sector = info->Ubeta >= 0.0F ? 1 : 0;
    sector = temp >= info->Ubeta ? sector | 0x02 : sector;
    sector = -temp >= info->Ubeta ? sector | 0x04 : sector;

    //计算时间比例
    switch (sector)
    {
    case 3: //第一扇区
        Tx = (1.5F * info->Ualpha - SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
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
        Tx = (1.5F * info->Ualpha + SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = (-1.5F * info->Ualpha + SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
        Tx = SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
        Ty = (-1.5F * info->Ualpha - SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
    case 4: //第四扇区
        Tx = (-1.5F * info->Ualpha + SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = -SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
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
        Tx = (-1.5F * info->Ualpha - SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = (1.5F * info->Ualpha - SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
    case 2: //第六扇区
        Tx = -SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
        Ty = (1.5F * info->Ualpha + SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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

/// @brief 将电机信息应用到电机
/// @param info 电机信息
void ApplyMotorInfo(MotorInfo* info)
{
    // 防止超调
    float ScaleFactor = 0.80F * SQRT3_DIV_2 / sqrtf(info->Uq * info->Uq + info->Ud * info->Ud);
    if (ScaleFactor < _2_DIV_3 / info->Udc)
    {
        info->Uq *= ScaleFactor;
        info->Ud *= ScaleFactor;
    }

    fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin值和cos值
    Rev_Park_Transf(info);  // 逆Park变换
    SVPWM(info);    // SVPWM
    setPWM(info);   // 设置PWM
}

/// @brief 检测电机电阻
/// @param info 电机信息
/// @param DetectingCurrent 检测电流(A)
/// @param MaxDetectingVoltage 最大检测电压(V)
/// @return 0:检测未完成 1:检测完成 -1:检测失败
int8_t DetectingResistance(MotorInfo* info, float DetectingCurrent, float MaxDetectingVoltage)
{
    static int8_t Flag = 0, ret = 0;
    static uint32_t LoopCount = 0;  //循环执行次数
    switch (Flag)
    {
    case 0: // 初始化参数
        LoopCount = 0;
        info->Direction = 1;
        info->Ud = 0.0F;
        info->Uq = 0.0F;
        info->Angle = 0.0F;
        OpenPWM( );
        Flag = 1;
        ret = 0;
        break;
    case 1:
        info->Ud += (DETECTINGRESISTANCE_KI * FOC_CONTROL_PERIOD) * (DetectingCurrent - ABS(info->Ib));   //电压积分
        if (info->Ud > MaxDetectingVoltage)
        {/// 电压超出限制
            ClosePWM( );
            Flag = 0;
            ret = -1;
            break;
        }
        ApplyMotorInfo(info);   //将电机信息应用到电机
        if (LoopCount++ >= FOC_CONTROL_FREQ * 6)
        {/// 到达最大检测时间
            ClosePWM( );
            if ((DetectingCurrent - ABS(info->Ib)) / DetectingCurrent > 0.1F)
                ret = -2;   /// 电流误差大于10%
            else
            {
                info->Resistance = info->Ud / DetectingCurrent;
                ret = 1;
            }
            info->Ud = 0.0F;
            Flag = 0;
        }
        break;
    default:
        info->Ud = 0.0F;
        Flag = 0;
        ret = -1;
        ClosePWM( );
        break;
    }
    return ret;
}

/// @brief 检测电机电感
/// @param info 电机信息
/// @param DetectingVoltage 检测电压(V)
/// @return 0:检测未完成 1:检测完成 -1:检测失败
int8_t DetectingInductance(MotorInfo* info, float DetectingVoltage)
{
    static float Current[2] = { 0.0F, 0.0F };
    static int8_t State = 0, ret = 0;
    static uint32_t LoopCount = 0;
    switch (State)
    {
    case 0:/// 初始化参数
        Current[0] = 0.0F;
        Current[1] = 0.0F;
        info->Direction = 1;
        info->Ud = -DetectingVoltage;
        info->Uq = 0.0F;
        info->Angle = 0.0F;
        OpenPWM( );
        ApplyMotorInfo(info);
        LoopCount = 0;
        State = 1;
        ret = 0;
        break;
    case 1:
        if (LoopCount++ & 0x01)
        {
            Current[0] += info->Ib;
            info->Ud = -DetectingVoltage;
        }
        else
        {
            Current[1] += info->Ib;
            info->Ud = DetectingVoltage;
        }
        ApplyMotorInfo(info);
        if (LoopCount >= FOC_CONTROL_FREQ)
        {/// 达到最大检测时间
            ClosePWM( );
            float dI_DIV_dt = (Current[1] - Current[0]) / (FOC_CONTROL_PERIOD * (LoopCount + 1));
            info->Inductance = ABS(DetectingVoltage / dI_DIV_dt);
            State = 0;
            ret = 1;
        }
        break;
    }
    return ret;
}


