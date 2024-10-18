#ifndef __ADC_H
#define __ADC_H

#include "Motor.h"

typedef enum
{
    _U_Value = 0,
    _V_Value = 1,
    _W_Value = 2,
    _Power_Value = 3,
    _Angle_Value = 4,
}ADC_Channel;

void adc_config(void);  //ADC初始化配置
void adc_OffsetConfig(void);  //初始化电机U、V、W相电流和电源电压偏置
void getMotorCurrent(MotorInfo* motor);  //获取电机U、V、W相电流
void getPowerVoltageAndTemp(float* PowerVoltage, float* Temp);  //获取电源电压

#endif /* __ADC_H */


