#ifndef FILTER_H
#define FILTER_H

#include  <stdint.h>

typedef struct
{
    float a;    //滤波系数
    float _1_a; //1-a
    float lastValue;    //上一次滤波值
}FirstOrderFilterInfo;  //一阶滤波器参数

typedef struct
{
    uint8_t N;  //滑动窗口大小
    float valueSum;  //采样值和
    float lastFilterValue;  //上一次滤波值
    uint8_t curNum; //当前采样值个数
}MoveAverageFilterInfo;   //移动平均滤波器参数

typedef struct
{
    uint8_t N;  //滑动窗口大小
    float MaxValue;  //最大幅度阈值
    float MinValue;  //最小幅度阈值
    float valueSum;  //采样值和
    float lastFilterValue;  //上一次滤波值
    uint8_t curNum; //当前采样值个数
    float (*getValue)(void);    //获取采样值的函数
}LAverageFilterInfo;    //限幅滑动平均滤波器参数

typedef struct
{
    float p;            //p控制滤波效果
    float q;            //q控制误差
    float r;            //r控制响应速度 
    float kGain;        //卡尔曼增益
    float lastValue;    //先前数值
}KalmanFilterInfo;  //卡尔曼滤波器参数

float FirstOrderFilter(FirstOrderFilterInfo* info, float Value);    //一阶滤波器
float MiddleValueFilter(uint8_t N, float (*getValue)(void));    //中值滤波器
float AverageFilter(uint8_t N, float (*getValue)(void));    //均值滤波器
float MoveAverageFilter(MoveAverageFilterInfo* info, float NewValue);    //移动平均滤波
float LAverageFilter(LAverageFilterInfo* info);    //限幅滑动平均滤波
float KalmanFilter(KalmanFilterInfo* info, float curValue);    //卡尔曼滤波

#endif // !FILTER_H


