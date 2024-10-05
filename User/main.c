#include "main.h"
#include "gd32f30x.h"
#include "systick.h"
#include "Delay.h"
#include "i2c.h"
#include "Encoder.h"
#include "ADC.h"
#include "Timer.h"
#include "Filter.h"
#include "Motor.h"
#include "PID.h"
#include "usart.h"
#include "fast_sin.h"
#include "math.h"
#include "Flash.h"
#include "stdlib.h"
#include "string.h"
#include "MMPrintf.h"

#define CONFIG_CHANGED_FLAG_ADDR FLASH_USER_START_ADDR	//配置改变标志位地址
#define SYSTEM_CONFIG_INFO_ADDR (CONFIG_CHANGED_FLAG_ADDR + 1)	//系统配置信息地址
#define _2PI 6.2831853F	//2PI

#define _PI 3.1415927F
// #define _4PI_5 2.5132741F	//4PI/5
// #define _3PI_2 4.7123889F	//3PI/2
// #define _PI_10 0.3141593F	//PI/10
// #define _19PI_10 5.9690260F	//19PI/10
// #define _PI_2 1.5707963F	//PI/2
// #define _PI_4 0.7853982F	//PI/4

SystemConfigInfo const* const SystemInfo = (SystemConfigInfo*)(SYSTEM_CONFIG_INFO_ADDR);	//系统配置信息存储地址
uint8_t const* const ConfigChangedFlag = (const uint8_t*)(CONFIG_CHANGED_FLAG_ADDR);	//配置改变标志位(为0代表配置已经被更新过)

FirstOrderFilterInfo FilterInfo_U = { 0.8F,0.2F, 0 };	//电机U相电流一阶互补滤波器参数
FirstOrderFilterInfo FilterInfo_W = { 0.8F,0.2F, 0 };	//电机W相电流一阶互补滤波器参数
FirstOrderFilterInfo FilterInfo_Power = { 0.8F,0.2F, 0 };	//电源电压一阶互补滤波器参数

MotorInfo motor = {
	.mode = MM_NULL,
	.MotorID = 0x01,
	.PolePairs = 7,
	.MAXPulse = 1000,
	.Direction = 1,
	.Resistance = 0.0F,
	.Inductance = 0.0F,
	.DetectingCurrent = 0.4F,
	.MaxDetectingVoltage = 6.0F,
	.Udc = 0.0F,
	.TargetCurrent = 0.0F,
	.TargetSpeed = 0.0F,
	.TargetPosition = 0.0F,
	.Uq = 0.0F,
	.Ud = 0.0F,
};

// 获取电机U相电流滤波后的值
static inline float get_U_Value(void) { return FirstOrderFilter(&FilterInfo_U, adc_getRawValue(_U_Value)); }
// 获取电机W相电流滤波后的值
static inline float get_W_Value(void) { return FirstOrderFilter(&FilterInfo_W, adc_getRawValue(_W_Value)); }
// 获取电源电压滤波后的值
static inline float get_Power_Value(void) { return FirstOrderFilter(&FilterInfo_Power, adc_getRawValue(_Power_Value)); }

/// 从Flash中读取系统配置信息,并更新给电机 
void MotorInfoUpdate(void)
{
	if (*ConfigChangedFlag == 0xFF) return;	//配置未改变
	//更新系统配置信息
	motor.MotorID = SystemInfo->MotorID;
	motor.PolePairs = SystemInfo->PolePairs;
	motor.Direction = SystemInfo->Direction;
	motor.Resistance = SystemInfo->Resistance;
	motor.Inductance = SystemInfo->Inductance;
	motor.DetectingCurrent = SystemInfo->DetectingCurrent;
	motor.MaxDetectingVoltage = SystemInfo->MaxDetectingVoltage;
	motor.mode = SystemInfo->MotorMode;
	Encoder.OffsetCount = SystemInfo->EncoderOffset;
	motor.MaxSpeed = SystemInfo->MaxSpeed;
}

/// 更新Flash中的系统配置信息
void SystemConfigInfoUpdate(void)
{
	SystemConfigInfo info = {
		.MotorID = motor.MotorID,
		.PolePairs = motor.PolePairs,
		.Direction = motor.Direction,
		.Resistance = motor.Resistance,
		.Inductance = motor.Inductance,
		.DetectingCurrent = motor.DetectingCurrent,
		.MaxDetectingVoltage = motor.MaxDetectingVoltage,
		.MotorMode = motor.mode,
		.EncoderOffset = Encoder.OffsetCount,
		.MaxSpeed = motor.MaxSpeed,
	};
	size_t size = sizeof(SystemConfigInfo) + 1;	// 字节数
	if (size % 4 != 0) size += 4 - size % 4;	// 4字节对齐
	uint32_t* data = (uint32_t*)malloc(size);	// 分配内存
	data[0] = 0;	// 配置已经改变
	memcpy((uint8_t*)data + 1, &info, sizeof(SystemConfigInfo));
	FMCErasePage(FLASH_USER_START_ADDR, 1);	//擦除Flash页
	FMCWriteData(FLASH_USER_START_ADDR, data, size / 4);	//写入Flash
}

uint8_t PrintfConfigInfoFlag = 0;	//打印配置信息标志位
/// 打印配置信息
void PrintfConfigInfo(void)
{
	MM_printf("MotorID:%u\n", motor.MotorID);
	MM_printf("PolePairs:%u\n", motor.PolePairs);
	MM_printf("Direction:%s\n", motor.Direction == 1 ? "CW" : motor.Direction == -1 ? "CCW" : "ERROR");
	MM_printf("Udc:%.6f\n", motor.Udc);
	MM_printf("Resistance:%.6f\n", motor.Resistance);
	MM_printf("Inductance:%.6f\n", motor.Inductance);
	MM_printf("DetectingCurrent:%.6f\n", motor.DetectingCurrent);
	MM_printf("MaxDetectingVoltage:%.6f\n", motor.MaxDetectingVoltage);
	MM_printf("MotorMode:%d\n", motor.mode);
	MM_printf("EncoderOffset:%u\n", Encoder.OffsetCount);
	MM_printf("MaxSpeed:%.6f\n", motor.MaxSpeed);
}

int main( )
{
	MotorInfoUpdate( );	// 从Flash中读取系统配置信息,并更新给电机
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);   // 设置中断分组
	systick_config( );
	i2c0_config( );	//I2C0配置
	usart_config( );	//串口配置
	adc_config( );	//ADC0配置，对U、V、W相电流和电源电压进行采样
	timer_config(6, 1000);	//PWM频率(中央对齐模式,频率减半)--(120MHz / 2 / 2 / 1000 = 30kHz)(电流环频率 = 30kHz / 3 = 10kHz)
	Delay_ms(100);	//延时100ms

	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

	adc_OffsetConfig( );	//初始化电机U、W相电流和电源电压偏置
	motor.Udc = get_Power_Value( );	//获取电源电压
	UpdatePIDInfo(&motor);	//更新PID参数
	Delay_ms(10);	//延时10ms
	nvic_irq_enable(ADC0_1_IRQn, 0, 1); // 使能ADC0_1中断
	while (1)
	{
		motor.Udc = get_Power_Value( );	//获取电源电压
		if (PrintfConfigInfoFlag == 1)
		{
			PrintfConfigInfo( );	//打印配置信息
			PrintfConfigInfoFlag = 0;
		}
		JustFloat_Show(10, motor.Ia, motor.Ib, motor.Ic, motor.TargetCurrent, motor.Iq, motor.Id, motor.TargetSpeed, Encoder.Speed, Encoder.Angle, Encoder.AccAngle);
	}
}

/*
PIDInfo PIDInfo_Current_IQ =
{
	.Kp = 0.2F,
	.Ki = 0.01F,
	.Kd = 0.0F,
	.maxIntegral = 10.0F,
	.maxOutput = 1.2F,
};	//电流环PID参数
PIDInfo PIDInfo_Current_ID =
{
	.Kp = 0.2F,
	.Ki = 0.01F,
	.Kd = 0.0F,
	.maxIntegral = 10.0F,
	.maxOutput = 1.2F,
};	//电流环PID参数
PIDInfo PIDInfo_Speed =
{
	.Kp = 0.01F,
	.Ki = 0.0001F,
	.Kd = 0.0F,
	.maxIntegral = 1000.0F,
	.maxOutput = 1.2F,
};	//电流环PID参数
float target = 0.0F;
void CurrentControl(MotorInfo* motor, float target_Iq, float angle)	//电流环控制
{
	motor->Angle = angle * motor->PolePairs;	//设置电机电角度
	motor->Angle = fmodf(motor->Angle, _2PI);
	motor->Ia = get_U_Value( );	//获取电机U相电流
	// motor->Ib = get_V_Value( );	//获取电机V相电流
	motor->Ic = get_W_Value( );	//获取电机W相电流
	Clarke_Transf(motor);	//Clarke变换
	Park_Transf(motor);	//Park变换
	PID_SingleCalc(&PIDInfo_Current_IQ, target_Iq, motor->Iq);	//IQ控制
	motor->Uq = PIDInfo_Current_IQ.output;
	//motor->Uq = 0.0F;
	PID_SingleCalc(&PIDInfo_Current_ID, 0, motor->Id);	//ID控制
	motor->Ud = PIDInfo_Current_ID.output;
	Rev_Park_Transf(motor);	//逆Park变换
	SVPWM(motor);	//SVPWM
	// setPWM(motor);	//设置PWM
}
*/

void SpeedOpenloop(float targetSpeed)
{
	// static float targetAngle = 0.0F;

	motor.Ud = 0.0F;
	motor.Uq = 2.0F;
	//计算累积目标角度
	// targetAngle += targetSpeed * FOC_CONTROL_PERIOD * motor.PolePairs;
	motor.Angle = (-Encoder.AccAngle + targetSpeed * FOC_CONTROL_PERIOD) * motor.PolePairs;	//设置电机角度
	motor.Angle = fmodf(motor.Angle, _2PI);	//限制在-2PI~2PI之间
	// motor.Angle = motor.Angle >= 0.0F ? motor.Angle : motor.Angle + _2PI;//限制在0~2PI之间
	// fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); //计算角度sin值和cos值
	// Clarke_Transf(&motor);	//Clarke变换
	// Park_Transf(&motor);	//Park变换
	// PIDSingleCalc(&motor.PIDInfoIQ, 0.40F, motor.Iq);	// Iq控制
	// motor.Uq = motor.PIDInfoIQ.output;	//设置Uq
	// PIDSingleCalc(&motor.PIDInfoID, 0.0F, motor.Id);	// Id控制
	// motor.Ud = motor.PIDInfoID.output;	//设置Ud
	ApplyMotorInfo(&motor);	//将电机信息应用到电机
}


//ADC0中断服务函数
void ADC0_1_IRQHandler(void)
{
	static int ret = 0;

	if (adc_interrupt_flag_get(ADC1, ADC_INT_FLAG_EOIC) == RESET)
	{/// 出现未知中断
		nvic_irq_disable(ADC0_1_IRQn); // 关闭ADC0_1中断
		return;
	}
	adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);	//清除ADC中断标志

	gpio_bit_set(GPIOA, GPIO_PIN_1);	//测试用

	Encoder_UpdateValue( );	//更新编码器值
	motor.Ia = get_U_Value( );	//获取电机U相电流
	motor.Ic = get_W_Value( );	//获取电机W相电流
	motor.Ib = -motor.Ia - motor.Ic;	//获取电机V相电流

	switch (motor.mode)
	{
	case MM_NULL:
		ClosePWM( );	// 关闭PWM
		break;
	case MM_DetectingResistance:/// 检测电机电阻
		ret = DetectingResistance(&motor, 0.40F, 8.0F);
		if (ret == 1)
		{// 检测完成
			motor.mode = MM_NULL;
			UpdatePIDInfo(&motor);	//更新PID参数
		}
		else if (ret == -1) motor.mode = MM_Error;
		break;
	case MM_DetectingInductance:/// 检测电机电感
		ret = DetectingInductance(&motor, 6.0F);
		if (ret == 1)
		{// 检测完成
			motor.mode = MM_NULL;
			UpdatePIDInfo(&motor);	//更新PID参数
		}
		else if (ret == -1) motor.mode = MM_Error;
		break;
	case MM_EncoderCalibration:/// 编码器校准
		ret = EncoderOffsetCalibration(&motor);
		if (ret == 1) motor.mode = MM_NULL;
		else if (ret == -1) motor.mode = MM_Error;
		break;
	case MM_PositionControl:
		PIDSingleCalc(&motor.PIDInfoPosition, motor.TargetPosition, Encoder.AccCount);	// 位置环控制
		motor.TargetSpeed = motor.PIDInfoPosition.output;	//设置目标速度
	case MM_SpeedControl:
		PIDSingleCalc(&motor.PIDInfoSpeed, motor.TargetSpeed, Encoder.Speed);	// 速度环控制
		motor.TargetCurrent = motor.PIDInfoSpeed.output;	//设置目标电流
	case MM_CurrentControl:
		OpenPWM( );	// 开启PWM
		motor.Angle = Encoder.AccAngle * motor.PolePairs;	//设置电机电角度
		motor.Angle = fmodf(motor.Angle, _2PI);	//限制在-2PI~2PI之间
		fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); //计算角度sin值和cos值
		Clarke_Transf(&motor);	//Clarke变换
		Park_Transf(&motor);	//Park变换
		PIDSingleCalc(&motor.PIDInfoIQ, motor.TargetCurrent, motor.Iq);	// Iq控制
		motor.Uq = motor.PIDInfoIQ.output;	//设置Uq
		PIDSingleCalc(&motor.PIDInfoID, 0.0F, motor.Id);	// Id控制
		motor.Ud = motor.PIDInfoID.output;	//设置Ud
		ApplyMotorInfo(&motor);	//将电机信息应用到电机
		break;
	case MM_OpenLoop:
		OpenPWM( );	// 开启PWM
		SpeedOpenloop(30.0F);	// 开环速度控制
		break;
	case MM_Error:
	default:
		ClosePWM( );	// 关闭PWM
		break;

	}

	gpio_bit_reset(GPIOA, GPIO_PIN_1);	//测试用
}



