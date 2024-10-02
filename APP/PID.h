#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp, Ki, Kd;   //比例、积分、微分系数
    float maxIntegral;  //积分限幅
    float maxOutput;    //输出限幅
    float error, lastError; //误差、上次误差
    float integral;    //积分、积分限幅
    float output;    //输出、输出限幅
}PIDInfo;
void PID_SingleCalc(PIDInfo* info,const float target,const float Value);  //PID控制

#endif // !PID_H