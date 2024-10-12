#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp, Ki, Kd;   //比例、积分、微分系数
    float maxIntegral;  //积分限幅
    float maxOutput;    //输出限幅
    float error, lastError; //误差、上次误差
    float integral;    //积分
    float output;    //输出
    float lastOutput;    //上次输出
}PIDInfo;

typedef struct
{
    float Kp, Ki, Kd;   //比例、积分、微分系数
    float maxOutput;    //输出限幅
    float curError;     //当前误差
    float lastError;    //上次误差
    float llasError;    //上上次误差
    float output;    //输出
}IncPIDInfo;

void PIDSingleCalc(PIDInfo* info, const float target, const float Value);  //位置式PID控制
void IncPIDSingleCalc(IncPIDInfo* info, const float target, const float Value);  //增量式PID控制

#endif // !PID_H