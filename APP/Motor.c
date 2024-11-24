#include "Motor.h"
#include "fast_sin.h"
#include "gd32f30x.h"
#include "math.h"
#include "main.h"
#include "Encoder.h"
#include "MMPrintf.h"

#define _PI 3.1415927F
#define _2PI 6.2831853F
#define _SQRT3 1.73205078F   // sqrt(3)
#define _SQRT3_DIV_2 0.866025388F    // sqrt(3)/2
#define _SQRT3_DIV_3 0.57735026F    // sqrt(3)/3
#define _1_DIV_3 0.33333333F    // 1/3
#define _2_DIV_3 0.66666666F    // 2/3 

static char PWMState = -1; //PWM状态(0:关闭,1:开启)
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
    if (motor->Direction == 1)  // 正转
    {
        // 设置占空比
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, motor->PulseA); // U
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, motor->PulseB); // V
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, motor->PulseC); // W
    }
    else if (motor->Direction == -1)    // 反转
    {
        // 设置占空比
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, motor->PulseA); // U
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, motor->PulseC); // V
        timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, motor->PulseB); // W
    }
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 10); // ADC采样控制PWM
}

/// @brief Clarke变换
/// @param info 电机信息
void Clarke_Transf(MotorInfo* info)
{
    info->Ialpha = (2 * info->Ia - info->Ib - info->Ic) * _1_DIV_3;
    info->Ibeta = (info->Ib - info->Ic) * _SQRT3_DIV_3;   //0.57735026 = sqrt(3)/3
}

void Rev_Clarke_Transf(MotorInfo* info)
{
    info->Ua = info->Ualpha;
    info->Ub = -0.5f * info->Ualpha + _SQRT3_DIV_2 * info->Ubeta;  //0.8660254 = sqrt(3)/2
    info->Uc = -0.5f * info->Ualpha - _SQRT3_DIV_2 * info->Ubeta;
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
    temp = _SQRT3 * info->Ualpha;
    sector = info->Ubeta >= 0.0F ? 1 : 0;
    sector = temp >= info->Ubeta ? sector | 0x02 : sector;
    sector = -temp >= info->Ubeta ? sector | 0x04 : sector;

    //计算时间比例
    switch (sector)
    {
    case 3: //第一扇区
        Tx = (1.5F * info->Ualpha - _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = _SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
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
        Tx = (1.5F * info->Ualpha + _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = (-1.5F * info->Ualpha + _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
        Tx = _SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
        Ty = (-1.5F * info->Ualpha - _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
        Tx = (-1.5F * info->Ualpha + _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = -_SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
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
        Tx = (-1.5F * info->Ualpha - _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
        Ty = (1.5F * info->Ualpha - _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
        Tx = -_SQRT3 * info->Ubeta * info->MAXPulse / info->Udc;
        Ty = (1.5F * info->Ualpha + _SQRT3_DIV_2 * info->Ubeta) * info->MAXPulse / info->Udc;
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
    Rev_Park_Transf(info);  // 逆Park变换
    SVPWM(info);    // SVPWM
    LIMIT(info->PulseA, 0, info->MAXPulse);
    LIMIT(info->PulseB, 0, info->MAXPulse);
    LIMIT(info->PulseC, 0, info->MAXPulse);
    setPWM(info);   // 设置PWM
}

/// @brief 检测电机电阻
/// @param info 电机信息
/// @param DetectingCurrent 检测电流(A)
/// @param MaxDetectingVoltage 最大检测电压(V)
/// @param Flag 是否初始化参数标志位(0:不初始化, !0:初始化)(主要用于退出检测模式时恢复参数并返回结果)
/// @return 0:检测未完成 1:检测完成 -1:电压超出限制 -2:电流误差大于10% -3:未知错误 -4:参数错误
int8_t DetectingResistance(MotorInfo* info, float DetectingCurrent, float MaxDetectingVoltage, uint8_t Flag)
{
    static int8_t State = 0, ret = 0, Dir = 0;
    static uint32_t LoopCount = 0;  //循环执行次数

    if (Flag)
    {/// 退出检测模式进行参数恢复
        if (State != 0)
            info->Direction = Dir; // 恢复方向
        State = 0;
        return ret;
    }

    if (info == NULL) return (ret = -4);

    switch (State)
    {
    case 0: // 初始化参数
        LoopCount = 0;
        info->Ud = 0.0F;
        info->Uq = 0.0F;
        Dir = info->Direction;  // 保存方向
        info->Direction = 1;
        info->Angle = 0.0F;
        State = 1;
        ret = 0;
        break;
    case 1:
        info->Ud += (DETECTINGRESISTANCE_KI * FOC_CONTROL_PERIOD) * (DetectingCurrent - fabsf(info->Ib));   //电压积分
        if (info->Ud > MaxDetectingVoltage)
        {/// 电压超出限制
            ret = -1;
            break;
        }
        fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin
        ApplyMotorInfo(info);   //将电机信息应用到电机
        if (LoopCount++ >= FOC_CONTROL_FREQ * 6)
        {/// 到达最大检测时间
            if ((DetectingCurrent - fabsf(info->Ib)) / DetectingCurrent > 0.1F)
            {
                ret = -2;   /// 电流误差大于10%
            }
            else
            {/// 检测完成
                info->Resistance = info->Ud / DetectingCurrent;
                ret = 1;
            }
            info->Ud = 0.0F;
            info->Angle = 0.0F;
            info->Direction = Dir; // 恢复方向
        }
        break;
    default:
        info->Ud = 0.0F;
        ret = -3;
        break;
    }
    return ret;
}

/// @brief 检测电机电感
/// @param info 电机信息
/// @param DetectingVoltage 检测电压(V)
/// @param Flag 是否初始化参数标志位(0:不初始化, !0:初始化)(主要用于退出检测模式时恢复参数并返回结果)
/// @return 0:检测未完成 1:检测完成 -1:未知错误 -2:参数错误
int8_t DetectingInductance(MotorInfo* info, float DetectingVoltage, uint8_t Flag)
{
    static float Current[2] = { 0.0F, 0.0F };
    static int8_t State = 0, ret = 0, Dir = 0;
    static uint32_t LoopCount = 0;

    if (Flag)
    {/// 退出检测模式进行参数恢复
        if (State != 0)
            info->Direction = Dir; // 恢复方向
        info->Ud = 0.0F;
        State = 0;
        return ret;
    }
    if (info == NULL) return (ret = -2);

    switch (State)
    {
    case 0:/// 初始化参数
        Current[0] = 0.0F;
        Current[1] = 0.0F;
        info->Ud = -DetectingVoltage;
        info->Uq = 0.0F;
        Dir = info->Direction;
        info->Direction = 1;
        info->Angle = 0.0F;
        fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin
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
        fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin
        ApplyMotorInfo(info);
        if (LoopCount >= FOC_CONTROL_FREQ)
        {/// 达到最大检测时间
            float dI_DIV_dt = (Current[1] - Current[0]) / (FOC_CONTROL_PERIOD * (LoopCount + 1));
            info->Inductance = fabsf(DetectingVoltage / dI_DIV_dt);
            info->Ud = 0.0F;
            ret = 1;
        }
        break;
    default:
        ret = -1;
        break;
    }
    return ret;
}

#define EOC_NUM 4
#define CALIBRAATING_SPEED (4 * _PI)
/// @brief 编码器偏置校准
/// @param info 电机信息
/// @param Flag 是否初始化参数标志位(0:不初始化, !0:初始化)(主要用于退出校准模式时恢复参数并返回结果)
/// @return 0:校准未完成 1:校准完成 -1:编码器错误 -2:极对数错误 -3:未知错误 -4:参数错误
int8_t EncoderOffsetCalibration(MotorInfo* info, uint8_t Flag)
{
    static uint8_t state = 0;
    static int8_t Dir = 0, ret = 0;;
    static uint32_t LoopCount = 0;
    static int32_t Diff = 0, StartCount = 0;
    static float CalibratingVoltage = 0.0F, AccAngle = 0.0F, AccCount = 0;//校准电压,校准速度

    if (Flag)
    {/// 退出校准模式进行参数恢复
        info->Ud = 0.0F;
        state = 0;
        return ret;
    }
    if (info == NULL) return (ret = -4);

    switch (state)
    {
    case 0:
        ret = 0;
        state = 0;
        info->Ud = 0.0F;
        info->Direction = 1;
        info->Angle = 0.0F;
        AccAngle = 0.0F;
        AccCount = 0;
        CalibratingVoltage = info->MaxCurrent * info->Resistance * _1_DIV_3 * _1_DIV_3;
        info->Uq = CalibratingVoltage;
        fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin,cos
        ApplyMotorInfo(info);
        state = 1;
        break;
    case 1:
        if (LoopCount++ >= FOC_CONTROL_FREQ)
        {/// 先定位1s
            LoopCount = 0;
            StartCount = Encoder.AccCount;
            state = 2;
        }
        break;
    case 2: // 转动16PI
        AccAngle += CALIBRAATING_SPEED * FOC_CONTROL_PERIOD;
        info->Angle = fmodf(AccAngle, _2PI);
        fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin,cos
        ApplyMotorInfo(info);
        if (++LoopCount % 10 == 0) AccCount += Encoder.AccCount;
        if (LoopCount > FOC_CONTROL_FREQ * EOC_NUM)
        {
            LoopCount = 0;
            state = 3;
        }
        break;
    case 3: // 判断方向与极对数
        Diff = Encoder.AccCount - StartCount;
        if (Diff > 100) Dir = 1;    // 正转
        else if (Diff < -100) Dir = -1; // 反转
        else
        {// 编码器出错
            Dir = 0;
            ret = -1;
        }
        info->PolePairs = roundf((CALIBRAATING_SPEED * EOC_NUM / _2PI * ENCODER_PULSE) / ABS(Diff));
        if (info->PolePairs < 1 || info->PolePairs > 30)
        {
            ret = -2;
            info->PolePairs = 7;
        }
        state = 4;
        break;
    case 4: // 反转
        AccAngle -= CALIBRAATING_SPEED * FOC_CONTROL_PERIOD;
        info->Angle = fmodf(AccAngle, _2PI);
        fast_sin_cos(info->Angle, &info->sinValue, &info->cosValue); //计算角度sin
        ApplyMotorInfo(info);
        if (++LoopCount % 10 == 0) AccCount += Encoder.AccCount;
        if (LoopCount > FOC_CONTROL_FREQ * EOC_NUM)
        {
            LoopCount = 0;
            state = 5;
        }
        break;
    case 5: // 计算偏置
        Encoder.OffsetCount = (int32_t)((AccCount / (FOC_CONTROL_FREQ / 10.0F * EOC_NUM * 2) - StartCount) + 0.5F);
        info->Direction = Dir;
        LoopCount = 0;
        ret = 1;
        break;
    default:
        ret = -3;
        break;
    }
    return ret;
}

/// @brief 抗齿槽力矩校准
/// @param info 电机信息
/// @param Flag 是否初始化参数标志位(0:不初始化, !0:初始化)(主要用于退出校准模式时恢复参数并返回结果)
/// @return 0:校准未完成 1:校准完成 -1:校准超时 -2:未知错误
int8_t AnticoggingCalibration(MotorInfo* info, uint8_t Flag)
{
    static int8_t ret = 0;
    static uint8_t State = 0;
    static uint32_t index = 0, LoopCount = 0;
    static float Diff = 0.0F, Increment = 0.0F, Ki = 0.0F;

    if (Flag)
    {
        if (State != 0)
            info->PIDInfoSpeed.Ki = Ki; // 重置积分增益
        State = 0;
        return ret;
    }
    if (info == NULL) return (ret = -3);

    switch (State)
    {
    case 0:
        ret = 0;
        index = 0;
        LoopCount = 0;
        Diff = 0.0F;
        info->AnticoggingCalibratedFlag = 0;
        Increment = _2PI / ANTICOGING_TABLE_NUM;    // 每次增加的角度
        info->TargetPosition = Encoder.AccAngle - ((float)Encoder.RawCount / ENCODER_PULSE * _2PI);  // 让电机处于编码器的零点
        Ki = info->PIDInfoSpeed.Ki;
        State = 1;
        break;
    case 1:/// 正转一圈
        Diff = info->TargetPosition - Encoder.AccAngle; // 计算角度差
        if (fabsf(Diff) < Increment / 10.0F && fabsf(Encoder.Speed) < 0.05F && LoopCount > (FOC_CONTROL_FREQ / 10))
        {/// 到达目标位置
            info->AnticogongTorqueTable[index++] = info->PIDInfoSpeed.integral * info->PIDInfoSpeed.Ki; // 记录校准力矩
            MM_printf("index:%d, Torque:%.6f\n", index - 1, info->AnticogongTorqueTable[index - 1]);    // 打印校准力矩
            info->TargetPosition += Increment;  // 下一个目标位置
            LoopCount = 0;  // 重置计数
            info->PIDInfoSpeed.Ki = Ki; // 重置积分增益
        }
        else/// 未到达目标位置
        {
            LoopCount++;
            if (LoopCount % (FOC_CONTROL_FREQ / 10) == 0) info->PIDInfoSpeed.Ki += Ki * 0.01F; // 增加积分增益
        }
        if (LoopCount >= FOC_CONTROL_FREQ * 30)
        {/// 超时
            ret = -1;
        }
        if (index >= ANTICOGING_TABLE_NUM)  // 正转校准完成
        {
            index = ANTICOGING_TABLE_NUM - 1;
            LoopCount = 0;
            State = 2;
        }
        break;
    case 2:/// 反转一圈
        Diff = info->TargetPosition - Encoder.AccAngle; // 计算角度差
        if (fabsf(Diff) < Increment / 5.0F && fabsf(Encoder.Speed) < 0.1F && LoopCount > (FOC_CONTROL_FREQ / 10))
        {/// 到达目标位置
            info->AnticogongTorqueTable[index] = info->PIDInfoSpeed.integral * info->PIDInfoSpeed.Ki; // 记录校准力矩
            info->AnticogongTorqueTable[index--] /= 2.0F; // 平均校准力矩
            MM_printf("index:%d, Torque:%.6f\n", index + 1, info->AnticogongTorqueTable[index + 1]);    // 打印校准力矩
            info->TargetPosition -= Increment;  // 下一个目标位置
            LoopCount = 0;  // 重置计数
            info->PIDInfoSpeed.Ki = Ki; // 重置积分增益
        }
        else/// 未到达目标位置
        {
            LoopCount++;
            if (LoopCount % (FOC_CONTROL_FREQ / 10) == 0) info->PIDInfoSpeed.Ki += Ki * 0.01F; // 增加积分增益
        }
        if (LoopCount >= FOC_CONTROL_FREQ * 30)
        {/// 超时
            ret = -1;
        }
        if (index >= ANTICOGING_TABLE_NUM)  // 完成校准
        {
            LoopCount = 0;
            State = 3;
        }
        break;
    case 3:
        ret = 1;
        break;
    default:
        ret = -2;
        break;
    }
    return ret;
}

/// @brief 更新PID参数
/// @param info 电机信息
void UpdatePIDInfo(MotorInfo* info)
{
    /// 电流环PID参数
    info->PIDInfoIQ.Kp = (FOC_CONTROL_FREQ / 20 * info->Inductance) * 0.15F;
    info->PIDInfoIQ.Ki = (info->Resistance / info->Inductance / FOC_CONTROL_FREQ) * 0.15F;
    info->PIDInfoIQ.Kd = 0.0F;
    info->PIDInfoIQ.maxOutput = info->MaxCurrent * info->Resistance;
    if (info->PIDInfoIQ.Ki == 0.0F) info->PIDInfoIQ.maxIntegral = 0.0F;
    else
        info->PIDInfoIQ.maxIntegral = info->PIDInfoIQ.maxOutput * _2_DIV_3 / ABS(info->PIDInfoIQ.Ki);

    info->PIDInfoID.Kp = info->PIDInfoIQ.Kp;
    info->PIDInfoID.Ki = info->PIDInfoIQ.Ki;
    info->PIDInfoID.Kd = 0.0F;
    info->PIDInfoID.maxOutput = info->PIDInfoIQ.maxOutput;
    info->PIDInfoID.maxIntegral = info->PIDInfoIQ.maxIntegral;

    /// 速度环PID参数
    info->PIDInfoSpeed.maxOutput = info->MaxCurrent;
    if (info->PIDInfoSpeed.Ki == 0.0F) info->PIDInfoSpeed.maxIntegral = 0.0F;
    else
        info->PIDInfoSpeed.maxIntegral = info->PIDInfoSpeed.maxOutput * _2_DIV_3 / ABS(info->PIDInfoSpeed.Ki);

    /// 位置环PID参数
    info->PIDInfoPosition.maxOutput = info->MaxSpeed;
    if (info->PIDInfoPosition.Ki == 0.0F) info->PIDInfoPosition.maxIntegral = 0.0F;
    else
        info->PIDInfoPosition.maxIntegral = info->PIDInfoPosition.maxOutput * _2_DIV_3 / ABS(info->PIDInfoPosition.Ki);
}

/// @brief 检测电机信息有效性
/// @param info 电机信息
/// @param PrintfFlag 是否打印信息(0:不打印 1:打印)
void CheckMotorInfoVality(MotorInfo* info)
{
    // 检查极对数
    if (info->PolePairs < 1 || info->PolePairs > 30)
    {/// 极对数错误
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_PolePairsError;
        return;
    }
    else if (info->ErrorInfo == Error_PolePairsError)
    {
        info->ErrorInfo = Error_Null;
    }
    // 检查电阻
    if (info->Resistance < 0.01F || info->Resistance > 30.0F)
    {/// 电阻错误
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_ResistanceError;
        return;
    }
    if (info->ErrorInfo == Error_ResistanceError)
    {
        info->ErrorInfo = Error_Null;
    }
    // 检查电感
    if (info->Inductance < 0.0001F || info->Inductance > 1.0F)
    {
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_InductanceError;
        return;
    }
    else if (info->ErrorInfo == Error_InductanceError)
    {
        info->ErrorInfo = Error_Null;
    }
    // 检查转向
    if (info->Direction != 1 && info->Direction != -1)
    {
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_DirectionError;
        return;
    }
    else if (info->ErrorInfo == Error_DirectionError)
    {
        info->ErrorInfo = Error_Null;
    }
    // 低压检测
    if (info->Udc < 11.10F) // 电源电压低于11.10V
    {
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_PowerLowVoltage;
        return;
    }
    else if (info->ErrorInfo == Error_PowerLowVoltage)
    {
        info->ErrorInfo = Error_Null;
    }
    if (info->Udc < 11.40F)
        info->WarningInfo = Warning_PowerLowVoltage;
    else if (info->WarningInfo == Warning_PowerLowVoltage)
        info->WarningInfo = Warning_Null;

    // 高压检测
    if (info->Udc > 16.80F)    // 电源电压高于16.80V
    {
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_PowerHighVoltage;
        return;
    }
    else if (info->ErrorInfo == Error_PowerHighVoltage)
    {
        info->ErrorInfo = Error_Null;
    }
    
    // 温度检测
    if (info->Temp > 65)
    {
        info->NextMode = MM_Error;
        if (info->ErrorInfo == Error_Null)
            info->ErrorInfo = Error_TemperatureHigh;
        return;
    }
    else if (info->ErrorInfo == Error_TemperatureHigh)
    {
        info->ErrorInfo = Error_Null;
    }
    if (info->Temp > 55)
        info->WarningInfo = Warning_TemperatureHigh;
    else if (info->WarningInfo == Warning_TemperatureHigh)
        info->WarningInfo = Warning_Null;
}

