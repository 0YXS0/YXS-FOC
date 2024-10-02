#ifndef __ADC_H
#define __ADC_H

#include "gd32f30x.h"

typedef enum
{
    _U_Value = 0,
    _V_Value = 1,
    _W_Value = 2,
    _Power_Value = 3,
    _Angle_Value = 4,
}ADC_Channel;

void adc_config(void);  //ADC初始化配置
float adc_getRawValue(const ADC_Channel channel);  //获取ADC采样值
void adc_OffsetConfig(void);  //初始化电机U、V、W相电流和电源电压偏置

#endif /* __ADC_H */


