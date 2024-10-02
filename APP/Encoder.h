#ifndef ENCODER_H
#define ENCODER_H

typedef enum
{
    EVT_Angle = 1,   //原始角度
    EVT_AccAngle,   //累积角度
    EVT_Speed,      //速度
}EncoderValueType;  //编码器值类型

float Encoder_GetValue(EncoderValueType type);  //获取编码器值
void Encoder_UpdateValue(void);  //更新编码器值

#endif // !ENCODER_H


