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

#define _1_DIV_3 0.33333333F	// 1/3
#define _2_DIV_3 0.66666666F	// 2/3
#define _PI 3.1415927F	// PI
#define _2PI 6.2831853F	// 2PI
#define CONFIG_CHANGED_FLAG_ADDR FLASH_USER_START_ADDR	// 配置改变标志位地址
#define SYSTEM_CONFIG_INFO_ADDR (CONFIG_CHANGED_FLAG_ADDR + 1)	// 系统配置信息地址
#define ANTICOGING_TABLE_ADDR (FLASH_USER_START_ADDR + FLASH_BANK0_PAGE_SIZE)	// 抗齿槽力矩表地址

uint8_t PrintfConfigInfoFlag = 0;	// 打印配置信息标志位
uint8_t PrintfDebugInfoFlag = 1;	// 打印调试信息标志位
SystemConfigInfo const* const SystemInfo = (SystemConfigInfo*)(SYSTEM_CONFIG_INFO_ADDR);	// 系统配置信息存储地址
uint8_t const* const ConfigChangedFlag = (const uint8_t*)(CONFIG_CHANGED_FLAG_ADDR);	// 配置改变标志位(为0代表配置已经被更新过)

FirstOrderFilterInfo FilterInfo_U = { 0.8F,0.2F, 0 };	// 电机U相电流一阶互补滤波器参数
FirstOrderFilterInfo FilterInfo_W = { 0.8F,0.2F, 0 };	// 电机W相电流一阶互补滤波器参数
FirstOrderFilterInfo FilterInfo_Power = { 0.8F,0.2F, 0 };	// 电源电压一阶互补滤波器参数

MotorInfo motor = {
	.mode = MM_NULL,
	.MotorID = 0x01,
	.Direction = -1,
	.PolePairs = 7,
	.MAXPulse = 1000,
	.MaxCurrent = 0.8F,
	.MaxSpeed = 120.0F,
	.Resistance = 13.375898F,
	.Inductance = 0.006646F,
	// .DetectingCurrent = 0.4F,
	// .MaxDetectingVoltage = 6.0F,
	.Udc = 0.0F,
	.TargetCurrent = 0.0F,
	.TargetSpeed = 0.0F,
	.TargetPosition = 0.0F,
	.PIDInfoSpeed.Kp = 0.008F,
	.PIDInfoSpeed.Ki = 0.00005F,
	.PIDInfoPosition.Kp = 2.5F,
	.PIDInfoPosition.Ki = 0.00008F,
	// .AnticogongTorqueTable = NULL,
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
	if (*ConfigChangedFlag != 0) return;	// 未进行过配置更新
	//更新系统配置信息
	motor.MotorID = SystemInfo->MotorID;	// 电机ID
	motor.PolePairs = SystemInfo->PolePairs;	// 电机极对数
	motor.Direction = SystemInfo->Direction;	// 电机转向
	motor.Resistance = SystemInfo->Resistance;	// 电机电阻
	motor.Inductance = SystemInfo->Inductance;	// 电机电感
	// motor.DetectingCurrent = SystemInfo->DetectingCurrent;	// 检测电流
	// motor.MaxDetectingVoltage = SystemInfo->MaxDetectingVoltage;	// 最大检测电压
	motor.mode = SystemInfo->MotorMode;	// 电机模式
	Encoder.OffsetCount = SystemInfo->EncoderOffset;	// 编码器偏置
	motor.MaxCurrent = SystemInfo->MaxCurrent;	// 最大电流
	motor.MaxSpeed = SystemInfo->MaxSpeed;	// 最大速度
	motor.PIDInfoSpeed.Kp = SystemInfo->SpeedKp;	// 速度环P参数
	motor.PIDInfoSpeed.Ki = SystemInfo->SpeedKi;	// 速度环I参数
	motor.PIDInfoPosition.Kp = SystemInfo->PositionKp;	// 位置环P参数
	motor.PIDInfoPosition.Ki = SystemInfo->PositionKi;	// 位置环I参数
	// motor.AnticogongTorqueTable = (float*)calloc(ANTICOGING_TABLE_NUM, sizeof(float));	// 分配内存
	motor.AnticoggingCalibratedFlag = SystemInfo->AnticoggingCalibratedFlag;	// 抗齿槽力矩校准标志
	memcpy(motor.AnticogongTorqueTable, (float*)(ANTICOGING_TABLE_ADDR), ANTICOGING_TABLE_NUM * sizeof(float));	// 读取抗齿槽力矩表
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
		// .DetectingCurrent = motor.DetectingCurrent,
		// .MaxDetectingVoltage = motor.MaxDetectingVoltage,
		.MotorMode = motor.mode,
		.EncoderOffset = Encoder.OffsetCount,
		.MaxCurrent = motor.MaxCurrent,
		.MaxSpeed = motor.MaxSpeed,
		.SpeedKp = motor.PIDInfoSpeed.Kp,
		.SpeedKi = motor.PIDInfoSpeed.Ki,
		.PositionKp = motor.PIDInfoPosition.Kp,
		.PositionKi = motor.PIDInfoPosition.Ki,
		.AnticoggingCalibratedFlag = motor.AnticoggingCalibratedFlag,
	};
	/// 写基本信息到Flash
	size_t size = sizeof(SystemConfigInfo) + 1;	// 字节数
	if (size % 4 != 0) size += 4 - size % 4;	// 4字节对齐
	uint32_t* data = (uint32_t*)malloc(size);	// 分配内存
	data[0] = 0;	// 表示配置已经改变
	memcpy((uint8_t*)data + 1, &info, sizeof(SystemConfigInfo));
	FMCErasePage(FLASH_USER_START_ADDR, FLASH_USER_USE_PAGE_NUM);	// 擦除Flash页
	FMCWriteData(FLASH_USER_START_ADDR, data, size / 4);	// 写入Flash
	/// 将抗齿槽力矩表写入Flash
	FMCWriteData(ANTICOGING_TABLE_ADDR, (uint32_t*)motor.AnticogongTorqueTable, ANTICOGING_TABLE_NUM);

	free(data);	// 释放内存
}

/// 打印配置信息
void PrintfConfigInfo(void)
{
	MM_printf("MotorID:%d\n", motor.MotorID);
	MM_printf("PolePairs:%d\n", motor.PolePairs);
	MM_printf("Direction:%s\n", motor.Direction == 1 ? "CW" : (motor.Direction == -1 ? "CCW" : "ERROR"));
	MM_printf("Udc:%.6f\n", motor.Udc);
	MM_printf("Resistance:%.6f\n", motor.Resistance);
	MM_printf("Inductance:%.6f\n", motor.Inductance);
	// MM_printf("DetectingCurrent:%.6f\n", motor.DetectingCurrent);
	// MM_printf("MaxDetectingVoltage:%.6f\n", motor.MaxDetectingVoltage);
	MM_printf("AnticoggingCalibrated:%s\n", motor.AnticoggingCalibratedFlag == 1 ? "YES" : (motor.AnticoggingCalibratedFlag == 0 ? "NO" : "ERROR"));
	MM_printf("MotorMode:%d\n", motor.mode);
	MM_printf("EncoderOffset:%d\n", Encoder.OffsetCount);
	MM_printf("MaxCurrent:%.6f\n", motor.MaxCurrent);
	MM_printf("MaxSpeed:%.6f\n", motor.MaxSpeed);
	MM_printf("SpeedKp:%.6f\n", motor.PIDInfoSpeed.Kp);
	MM_printf("SpeedKi:%.6f\n", motor.PIDInfoSpeed.Ki);
	MM_printf("PositionKp:%.6f\n", motor.PIDInfoPosition.Kp);
	MM_printf("PositionKi:%.6f\n", motor.PIDInfoPosition.Ki);
}

int main( )
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);   // 设置中断分组
	systick_config( );
	i2c0_config( );	// I2C0配置
	usart_config( );	// 串口配置
	adc_config( );	// ADC0配置，对U、V、W相电流和电源电压进行采样
	timer_config(6, 1000);	// PWM频率(中央对齐模式,频率减半)--(120MHz / 2 / 2 / 1000 = 30kHz)(电流环频率 = 30kHz / 3 = 10kHz)
	Delay_ms(1);	// 延时1ms

	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

	adc_OffsetConfig( );	// 初始化电机U、W相电流和电源电压偏置
	MotorInfoUpdate( );	// 从Flash中读取系统配置信息,并更新给电机
	UpdatePIDInfo(&motor);	// 更新PID参数
	for (size_t i = 0; i < 1000; i++)
	{/// 确保的到稳定的电源电压(低通滤波器需要一定时间才能稳定)
		motor.Udc = get_Power_Value( );	// 获取电源电压
	}
	Encoder.AccCount = 0.0F;	// 累积计数清零
	if (motor.Resistance != 0.0F)
		motor.MaxCurrent = MIN(motor.MaxCurrent, motor.Udc / motor.Resistance);	// 限制最大电流
	Delay_ms(1);	// 延时1ms
	nvic_irq_enable(ADC0_1_IRQn, 0, 1); // 使能ADC0_1中断
	while (1)
	{
		motor.Udc = get_Power_Value( );	// 获取电源电压
		if (PrintfConfigInfoFlag == 1)
		{
			PrintfConfigInfo( );	// 打印配置信息
			PrintfConfigInfoFlag = 0;
		}
		if (PrintfDebugInfoFlag == 1)
		{
			JustFloat_Show(14, motor.Ia, motor.Ib, motor.Ic, motor.TargetCurrent, motor.Iq, motor.Id, motor.TargetSpeed, Encoder.Speed, Encoder.Angle, Encoder.AccAngle, motor.Uq, motor.Ud, motor.TargetPosition, (float)Encoder.RawCount);
		}
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
static float OpenLoopTargetSpeed = 30.0F;
void SpeedOpenloop(void)
{
	static float targetAngle = 0.0F;
	motor.Ud = 0.0F;
	motor.Uq = motor.MaxCurrent * motor.Resistance * _1_DIV_3;
	//计算累积目标角度
	targetAngle += OpenLoopTargetSpeed * FOC_CONTROL_PERIOD * motor.PolePairs;
	// motor.Angle = (Encoder.Angle + (30.0F) * FOC_CONTROL_PERIOD) * motor.PolePairs;	//设置电机角度
	motor.Angle = targetAngle;	//设置电机角度
	motor.Angle = fmodf(motor.Angle, _2PI);	//限制在-2PI~2PI之间
	// motor.Angle = motor.Angle >= 0.0F ? motor.Angle : motor.Angle + _2PI;//限制在0~2PI之间
	// fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); //计算角度sin值和cos值
	// Clarke_Transf(&motor);	//Clarke变换
	// Park_Transf(&motor);	//Park变换
	// PIDSingleCalc(&motor.PIDInfoIQ, 0.40F, motor.Iq);	// Iq控制
	// motor.Uq = motor.PIDInfoIQ.output;	//设置Uq
	// PIDSingleCalc(&motor.PIDInfoID, 0.0F, motor.Id);	// Id控制
	// motor.Ud = motor.PIDInfoID.output;	//设置Ud
	fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); // 计算角度sin值和cos值
	Clarke_Transf(&motor);	// Clarke变换
	Park_Transf(&motor);	// Park变换
	ApplyMotorInfo(&motor);	//将电机信息应用到电机
}

//ADC0中断服务函数
void ADC0_1_IRQHandler(void)
{
	static int ret = 0;
	static float OpenLoopTargetAngle = 0.0F;

	if (adc_interrupt_flag_get(ADC1, ADC_INT_FLAG_EOIC) == RESET)
	{/// 出现未知中断
		nvic_irq_disable(ADC0_1_IRQn); // 关闭ADC0_1中断
		return;
	}
	adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);	// 清除ADC中断标志

	gpio_bit_set(GPIOA, GPIO_PIN_1);	// 测试用

	Encoder_UpdateValue( );	// 更新编码器值
	motor.Ia = get_U_Value( );	// 获取电机U相电流
	motor.Ic = get_W_Value( );	// 获取电机W相电流
	motor.Ib = -motor.Ia - motor.Ic;	// 获取电机V相电流

	switch (motor.mode)
	{
	case MM_NULL:
		ret = 0;
		OpenLoopTargetAngle = 0.0F;
		ClosePWM( );	// 关闭PWM
		break;

	case MM_DetectingResistance:/// 检测电机电阻
		ret = DetectingResistance(&motor, motor.MaxCurrent * _1_DIV_3, motor.Udc * _1_DIV_3);
		if (ret == 1)
		{// 检测完成
			motor.mode = MM_NULL;
			UpdatePIDInfo(&motor);	// 更新PID参数
		}
		else if (ret < 0) motor.mode = MM_Error;
		break;

	case MM_DetectingInductance:/// 检测电机电感
		ret = DetectingInductance(&motor, motor.MaxCurrent * motor.Resistance * _1_DIV_3);
		if (ret == 1)
		{// 检测完成
			motor.mode = MM_NULL;
			UpdatePIDInfo(&motor);	// 更新PID参数
		}
		else if (ret < 0) motor.mode = MM_Error;
		break;

	case MM_EncoderCalibration:/// 编码器校准
		ret = EncoderOffsetCalibration(&motor);
		if (ret == 1) motor.mode = MM_NULL;
		else if (ret < 0) motor.mode = MM_Error;
		break;

	case MM_OpenLoop:	// 开环控制
		motor.TargetCurrent = 0.3F;
		OpenLoopTargetAngle += OpenLoopTargetSpeed * FOC_CONTROL_PERIOD * motor.PolePairs;
		motor.Angle = fmodf(OpenLoopTargetAngle, _2PI);	//限制在-2PI~2PI之间
		goto CurrentControl;

	case MM_AnticoggingCalibration:	// 抗齿槽力矩校准
		ret = AnticoggingCalibration(&motor);
		if (ret == 1) motor.mode = MM_NULL;
		else if (ret < 0) motor.mode = MM_Error;
	case MM_PositionControl:
		PIDSingleCalc(&motor.PIDInfoPosition, motor.TargetPosition, Encoder.AccAngle);	// 位置环控制
		motor.TargetSpeed = motor.PIDInfoPosition.output;	// 设置目标速度
	case MM_SpeedControl:
		PIDSingleCalc(&motor.PIDInfoSpeed, motor.TargetSpeed, Encoder.Speed);	// 速度环控制
		motor.TargetCurrent = motor.PIDInfoSpeed.output;	// 设置目标电流
	case MM_CurrentControl:	// 电流环控制
		motor.Angle = Encoder.Angle * motor.PolePairs;	// 设置电机电角度
		motor.Angle = fmodf(motor.Angle, _2PI);	// 限制在-2PI~2PI之间
CurrentControl:
		OpenPWM( );	// 开启PWM
		fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); // 计算角度sin值和cos值
		Clarke_Transf(&motor);	// Clarke变换
		Park_Transf(&motor);	// Park变换
		if (motor.mode != MM_AnticoggingCalibration && motor.AnticoggingCalibratedFlag == 1)// 抗齿槽力矩补偿
			motor.TargetCurrent += motor.AnticogongTorqueTable[(uint32_t)roundf((float)Encoder.RawCount / ANTICOGING_INCREMENT)];
		PIDSingleCalc(&motor.PIDInfoIQ, motor.TargetCurrent, motor.Iq);	// Iq控制
		motor.Uq = motor.PIDInfoIQ.output;	// 设置Uq
		PIDSingleCalc(&motor.PIDInfoID, 0.0F, motor.Id);	// Id控制
		motor.Ud = motor.PIDInfoID.output;	// 设置Ud
		ApplyMotorInfo(&motor);	// 将电机信息应用到电机
		break;

	case MM_Error:
	default:
		ret = 0;
		ClosePWM( );	// 关闭PWM
		break;
	}

	gpio_bit_reset(GPIOA, GPIO_PIN_1);	// 测试用
}



