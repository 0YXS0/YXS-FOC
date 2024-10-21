#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "gd32f30x.h"
#include "systick.h"
#include "Delay.h"
#include "i2c.h"
#include "SPI.h"
#include "Encoder.h"
#include "ADC.h"
#include "Timer.h"
#include "Motor.h"
#include "usart.h"
#include "fast_sin.h"
#include "Flash.h"
#include "MMPrintf.h"

#define _1_DIV_3 0.33333333F	// 1/3
#define _2_DIV_3 0.66666666F	// 2/3
#define _PI 3.1415927F	// PI
#define _2PI 6.2831853F	// 2PI
#define CONFIG_CHANGED_FLAG_ADDR FLASH_USER_START_ADDR	// 配置改变标志位地址
#define SYSTEM_CONFIG_INFO_ADDR (CONFIG_CHANGED_FLAG_ADDR + 1)	// 系统配置信息地址
#define ANTICOGING_TABLE_ADDR (FLASH_USER_START_ADDR + FLASH_BANK0_PAGE_SIZE)	// 抗齿槽力矩表地址

uint8_t PrintfConfigInfoFlag = 0;	// 打印配置信息标志位
uint8_t PrintfDebugInfoFlag = 0;	// 打印调试信息标志位
SystemConfigInfo const* const SystemInfo = (SystemConfigInfo*)(SYSTEM_CONFIG_INFO_ADDR);	// 系统配置信息存储地址
uint8_t const* const ConfigChangedFlag = (const uint8_t*)(CONFIG_CHANGED_FLAG_ADDR);	// 配置改变标志位(为0代表配置已经被更新过)

MotorInfo motor = {
	.CurMode = MM_NULL,
	.MotorID = 0x01,
	.Direction = 1,
	.PolePairs = 7,
	.MAXPulse = TIMER_PERIOD,
	.MaxCurrent = 10.0F,
	.MaxSpeed = 120.0F,
	.Resistance = 0.232926F,
	.Inductance = 0.000147F,
	.Udc = 0.0F,
	.TargetCurrent = 0.0F,
	.TargetSpeed = 0.0F,
	.TargetPosition = 0.0F,
	.PIDInfoSpeed.Kp = 0.0F,
	.PIDInfoSpeed.Ki = 0.0F,
	.PIDInfoPosition.Kp = 0.0F,
	.PIDInfoPosition.Ki = 0.0F,
	.AnticoggingCalibratedFlag = 0,
	.WarningInfo = Warning_Null,
	.ErrorInfo = Error_Null,
};

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
	motor.NextMode = SystemInfo->MotorMode;	// 电机模式
	Encoder.OffsetCount = SystemInfo->EncoderOffset;	// 编码器偏置
	motor.MaxCurrent = SystemInfo->MaxCurrent;	// 最大电流
	motor.MaxSpeed = SystemInfo->MaxSpeed;	// 最大速度
	motor.PIDInfoSpeed.Kp = SystemInfo->SpeedKp;	// 速度环P参数
	motor.PIDInfoSpeed.Ki = SystemInfo->SpeedKi;	// 速度环I参数
	motor.PIDInfoPosition.Kp = SystemInfo->PositionKp;	// 位置环P参数
	motor.PIDInfoPosition.Ki = SystemInfo->PositionKi;	// 位置环I参数
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
		.MotorMode = motor.CurMode,
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
	MM_printf("SystemConfigInfoUpdate-Success\n");
}

/// @brief 获取警告信息字符串
char* getWarningStr(WarningType Type)
{
	switch (Type)
	{
	case Warning_Null:
		return "Null";
	case Warning_PowerLowVoltage:
		return "PowerLowVoltage";
	case Warning_TemperatureHigh:
		return "TemperatureHigh";
	}
	return "";
}

/// @brief 获取错误信息字符串
char* getErrorStr(ErrorType Type)
{
	switch (Type)
	{
	case Error_Null:
		return "Null";
	case Error_EncoderReadError:
		return "EncoderReadError";
	case Error_PowerLowVoltage:
		return "PowerLowVoltage";
	case Error_PowerHighVoltage:
		return "PowerHighVoltage";
	case Error_PolePairsError:
		return "PolePairsError";
	case Error_ResistanceError:
		return "ResistanceError";
	case Error_InductanceError:
		return "InductanceError";
	case Error_DirectionError:
		return "DirectionError";
	case Error_MaxCurrentError:
		return "MaxCurrentError";
	case Error_DetectingResistance_Unknown:
		return "DetectingResistance-Unknown";
	case Error_DetectingResistance_OverVoltage:
		return "DetectingResistance-OverVoltage";
	case Error_DetectingResistance_LargeCurrentError:
		return "DetectingResistance-LargeCurrentError";
	case Error_DetectingInductance_Unknown:
		return "DetectingInductance-Unknown";
	case Error_EncoderOffsetCalibration_Unknown:
		return "EncoderOffsetCalibration-Unknown";
	case Error_EncoderOffsetCalibration_OverTime:
		return "EncoderOffsetCalibration-OverTime";
	case Error_EncoderOffsetCalibration_EncoderError:
		return "EncoderOffsetCalibration-EncoderError";
	case Error_EncoderOffsetCalibration_PolePairsError:
		return "EncoderOffsetCalibration-PolePairsError";
	case Error_Unknown:
	default:
		return "Unknown";
	}
}

/// 打印配置信息
void PrintfConfigInfo(void)
{
	MM_printf("MotorID:%d\n", motor.MotorID);
	MM_printf("PolePairs:%d\n", motor.PolePairs);
	MM_printf("Direction:%s\n", motor.Direction == 1 ? "CW" : (motor.Direction == -1 ? "CCW" : "ERROR"));
	MM_printf("Udc:%.6f\n", motor.Udc);
	MM_printf("Temp:%.6f\n", motor.Temp);
	MM_printf("Resistance:%.6f\n", motor.Resistance);
	MM_printf("Inductance:%.6f\n", motor.Inductance);
	MM_printf("AnticoggingCalibrated:%s\n", motor.AnticoggingCalibratedFlag == 1 ? "YES" : (motor.AnticoggingCalibratedFlag == 0 ? "NO" : "ERROR"));
	MM_printf("MotorMode:%d\n", motor.CurMode);
	MM_printf("EncoderOffset:%d\n", Encoder.OffsetCount);
	MM_printf("MaxCurrent:%.6f\n", motor.MaxCurrent);
	MM_printf("MaxSpeed:%.6f\n", motor.MaxSpeed);
	MM_printf("SpeedKp:%.6f\n", motor.PIDInfoSpeed.Kp);
	MM_printf("SpeedKi:%.6f\n", motor.PIDInfoSpeed.Ki);
	MM_printf("PositionKp:%.6f\n", motor.PIDInfoPosition.Kp);
	MM_printf("PositionKi:%.6f\n", motor.PIDInfoPosition.Ki);
	MM_printf("WarningInfo:%s\n", getWarningStr(motor.WarningInfo));
	MM_printf("ErrorInfo:%s\n", getErrorStr(motor.ErrorInfo));
}

int main( )
{
	uint32_t Value = 0;
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);   // 设置中断分组
	systick_config( );
	// i2c0_config( );	// I2C0配置
	SPI0_Config( );	// SPI0配置
	AS5047P_Init( );	// AS5047P初始化
	usart_config( );	// 串口配置
	adc_config( );	// ADC0配置，对U、V、W相电流和电源电压进行采样
	timer_config(TIMER_PRESCALER, TIMER_PERIOD);	// PWM频率(中央对齐模式,频率减半)--(120MHz / 3 / 2 / 1000 = 20kHz)
	Delay_ms(10);	// 延时10ms
	adc_OffsetConfig( );	// 初始化电机U、W相电流和电源电压偏置
	MotorInfoUpdate( );	// 从Flash中读取系统配置信息,并更新给电机
	UpdatePIDInfo(&motor);	// 更新PID参数

	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);	// 测试引脚

	if (getEncoderRawCount(&Value) < 0)	// 编码器读取错误
		motor.ErrorInfo = Error_EncoderReadError;
	else
		nvic_irq_enable(ADC0_1_IRQn, 0, 1); // 使能ADC0_1中断
	while (1)
	{
		getPowerVoltageAndTemp(&motor.Udc, &motor.Temp);	// 获取电源电压
		CheckMotorInfoVality(&motor, 0);	// 检测电机信息有效性并更新PID参数
		if (motor.Udc < 11.40F)
			motor.WarningInfo = Warning_PowerLowVoltage;
		else if (motor.WarningInfo == Warning_PowerLowVoltage)
			motor.WarningInfo = Warning_Null;

		if (PrintfConfigInfoFlag == 1)
		{/// 打印配置信息
			PrintfConfigInfo( );
			PrintfConfigInfoFlag = 0;
		}
		if (PrintfDebugInfoFlag == 1)
		{/// 打印调试信息
			JustFloat_Show(14, motor.Ia, motor.Ib, motor.Ic, motor.TargetCurrent, motor.Iq, motor.Id, motor.TargetSpeed, Encoder.Speed, Encoder.Angle, Encoder.AccAngle, motor.Uq, motor.Ud, motor.TargetPosition, (float)Encoder.RawCount);
		}
	}
}

/// @brief 退出当前电机模式
static inline void QuitMotorMode(void)
{
	int8_t ret = 0;	// 返回值

	switch (motor.CurMode)
	{
	case MM_NULL:	// 退出空模式
		break;
	case MM_DetectingResistance:// 退出电机电阻检测模式
		ret = DetectingResistance(&motor, 0, 0, True);	// 获取电阻检测结果并初始化检测参数
		if (ret == 0)
		{/// 电阻检测未完成
			motor.Resistance = 0.0F;
			motor.ErrorInfo = Error_ResistanceError;
			MM_printf("MotorResistanceError\n");
		}
		else if (ret == 1)
		{/// 电阻检测完成
			if (motor.ErrorInfo == Error_DetectingInductance_Unknown ||
				motor.ErrorInfo == Error_DetectingResistance_LargeCurrentError ||
				motor.ErrorInfo == Error_DetectingResistance_OverVoltage)
				motor.ErrorInfo = Error_Null;
			CheckMotorInfoVality(&motor, True);	// 检测电机信息有效性
			UpdatePIDInfo(&motor);	// 更新PID参数
			MM_printf("DetectingResistance-Success\nResistance:%.6f\n", motor.Resistance);
		}
		else if (ret == -1)
		{/// 电压超过限制
			motor.Resistance = 0.0F;
			motor.ErrorInfo = Error_DetectingResistance_OverVoltage;
			MM_printf("DetectingResistance-Voltage exceeds the limit\n");
		}
		else if (ret == -2)
		{/// 大电流错误
			motor.Resistance = 0.0F;
			motor.ErrorInfo = Error_DetectingResistance_LargeCurrentError;
			MM_printf("DetectingResistance-Large current error\n");
		}
		else
		{/// 电阻检测未知错误
			motor.Resistance = 0.0F;
			motor.ErrorInfo = Error_DetectingResistance_Unknown;
			MM_printf("DetectingResistance-Unknown error\n");
		}
		break;
	case MM_DetectingInductance:// 退出电机电感检测模式
		ret = DetectingInductance(&motor, 0, True);	// 获取电感检测结果并初始化检测参数
		if (ret == 0)
		{/// 电感检测未完成
			motor.ErrorInfo = Error_InductanceError;
			MM_printf("MotorInductanceError\n");
		}
		else if (ret == 1)
		{/// 电感检测完成
			if (motor.ErrorInfo == Error_DetectingInductance_Unknown)
				motor.ErrorInfo = Error_Null;
			CheckMotorInfoVality(&motor, True);	// 检测电机信息有效性
			UpdatePIDInfo(&motor);	// 更新PID参数
			MM_printf("DetectingInductance-Success\nInductance:%.6f\n", motor.Inductance);
		}
		else
		{/// 电感检测未知错误
			motor.ErrorInfo = Error_DetectingInductance_Unknown;
			MM_printf("DetectingInductance-Unknown error\n");
		}
		break;
	case MM_EncoderCalibration:	// 退出编码器校准模式
		ret = EncoderOffsetCalibration(&motor, True);	// 编码器校准
		if (ret == 0)
		{
			Encoder.OffsetCount = 0;
			motor.PolePairs = 0;
			motor.Direction = 0;
			motor.ErrorInfo = Error_EncoderOffsetError;
			MM_printf("EncoderOffsetCalibration-Exit midway\n");
		}
		else if (ret == 1)
		{
			if (motor.ErrorInfo == Error_EncoderOffsetCalibration_EncoderError ||
				motor.ErrorInfo == Error_EncoderOffsetCalibration_PolePairsError ||
				motor.ErrorInfo == Error_EncoderOffsetCalibration_Unknown)
				motor.ErrorInfo = Error_Null;
			CheckMotorInfoVality(&motor, True);	// 检测电机信息有效性
			MM_printf("EncoderOffsetCalibration-Success\nDirection:%s\nOffsetCount:%d\nPolePairs:%d\n", motor.Direction == 1 ? "CW" : "CCW", Encoder.OffsetCount, motor.PolePairs);
		}
		else if (ret == -1)
		{
			motor.ErrorInfo = Error_EncoderOffsetCalibration_EncoderError;
			MM_printf("EncoderOffsetCalibration-EncoderError\n");
		}
		else if (ret == -2)
		{
			motor.ErrorInfo = Error_EncoderOffsetCalibration_PolePairsError;
			MM_printf("EncoderOffsetCalibration-PolePairs error\nPolePairs:%d\n", motor.PolePairs);
		}
		else
		{
			motor.ErrorInfo = Error_EncoderOffsetCalibration_Unknown;
			MM_printf("EncoderOffsetCalibration-Unknown error\n");
		}
		break;
	case MM_OpenLoopVF:
		motor.OpenLoopTargetAngle_VF = 0.0F;
		motor.OpenLoopTargetSpeed = 0.0F;
		break;
	case MM_OpenLoopIF:
		motor.OpenLoopTargetAngle_IF = 0.0F;
		motor.OpenLoopTargetSpeed = 0.0F;
		break;
	case MM_AnticoggingCalibration:
		ret = AnticoggingCalibration(&motor, True);
		if (ret == 0)
		{
			motor.AnticoggingCalibratedFlag = False;
			MM_printf("AnticoggingCalibration-Exit midway\n");
		}
		else if (ret == 1)
		{
			motor.AnticoggingCalibratedFlag = True;
			MM_printf("AnticoggingCalibration-Success\n");
		}
		else if (ret == -1)
		{
			MM_printf("AnticoggingCalibration-Unknown error\n");
		}
		else
		{
			MM_printf("AnticoggingCalibration-Unknown error\n");
		}
		break;
	case MM_PositionControl:
		motor.PIDInfoPosition.integral = 0.0F;	// 位置环积分清零
		motor.PIDInfoPosition.lastError = 0.0F;	// 位置环上一次误差清零
		motor.PIDInfoPosition.lastOutput = 0.0F;	// 位置环上一次输出清零
		break;
	case MM_SpeedControl:
		motor.PIDInfoSpeed.integral = 0.0F;	// 速度环积分清零
		motor.PIDInfoSpeed.lastError = 0.0F;	// 速度环上一次误差清零
		motor.PIDInfoSpeed.lastOutput = 0.0F;	// 速度环上一次输出清零
		motor.TargetSpeed = 0.0F;	// 目标速度清零
		break;
	case MM_CurrentControl:
		motor.PIDInfoIQ.integral = 0.0F;	// q轴电流环积分清零
		motor.PIDInfoIQ.lastError = 0.0F;	// q轴电流环上一次误差清零
		motor.PIDInfoIQ.lastOutput = 0.0F;	// q轴电流环上一次输出清零
		motor.PIDInfoID.integral = 0.0F;	// d轴电流环积分清零
		motor.PIDInfoID.lastError = 0.0F;	// d轴电流环上一次误差清零
		motor.PIDInfoID.lastOutput = 0.0F;	// d轴电流环上一次输出清零
		motor.TargetCurrent = 0.0F;	// 目标电流清零
		break;
	case MM_Error:
		if (motor.ErrorInfo == Error_Null)
			motor.CurMode = MM_NULL;
		break;
	}
	ClosePWM( );	// 关闭PWM
}

/// @brief 进入下一电机模式
static inline void EnterMotorMode(void)
{
	if (motor.CurMode != MM_Error && motor.ErrorInfo != Error_Null)	// 有错误信息但不处于错误模式,则将当前模式改为错误模式
		motor.CurMode = MM_Error;
	switch (motor.NextMode)
	{
	case MM_NULL:
		motor.CurMode = MM_NULL;
		break;
	case MM_DetectingResistance:
		if (motor.CurMode == MM_Error && motor.ErrorInfo != Error_ResistanceError) break;
		MM_printf("DetectingResistance-Start\n");
		motor.CurMode = MM_DetectingResistance;
		OpenPWM( );	// 开启PWM
		break;
	case MM_DetectingInductance:
		if (motor.CurMode == MM_Error && motor.ErrorInfo != Error_InductanceError) break;
		MM_printf("DetectingInductance-Start\n");
		motor.CurMode = MM_DetectingInductance;
		OpenPWM( );	// 开启PWM
		break;
	case MM_EncoderCalibration:
		if (motor.CurMode == MM_Error &&
			!(motor.ErrorInfo == Error_EncoderOffsetError || motor.ErrorInfo == Error_PolePairsError || motor.ErrorInfo == Error_DirectionError))
			break;
		MM_printf("EncoderOffsetCalibration-Start\n");
		motor.CurMode = MM_EncoderCalibration;
		OpenPWM( );	// 开启PWM
		break;
	case MM_OpenLoopVF:
		if (motor.CurMode == MM_Error) break;
		motor.CurMode = MM_OpenLoopVF;
		OpenPWM( );	// 开启PWM
		break;
	case MM_OpenLoopIF:
		if (motor.CurMode == MM_Error) break;
		motor.CurMode = MM_OpenLoopIF;
		OpenPWM( );	// 开启PWM
		break;
	case MM_AnticoggingCalibration:
		if (motor.CurMode == MM_Error) break;
		MM_printf("AnticoggingCalibration-Start\n");
		motor.CurMode = MM_AnticoggingCalibration;
		OpenPWM( );	// 开启PWM
		break;
	case MM_PositionControl:
		if (motor.CurMode == MM_Error) break;
		motor.CurMode = MM_PositionControl;
		motor.TargetPosition = Encoder.AccAngle;
		OpenPWM( );	// 开启PWM
		break;
	case MM_SpeedControl:
		if (motor.CurMode == MM_Error) break;
		motor.CurMode = MM_SpeedControl;
		OpenPWM( );	// 开启PWM
		break;
	case MM_CurrentControl:
		if (motor.CurMode == MM_Error) break;
		motor.CurMode = MM_CurrentControl;
		OpenPWM( );	// 开启PWM
		break;
	case MM_Error:
		motor.CurMode = MM_Error;
		break;
	}
}

void ADC0_1_IRQHandler(void)
{//ADC0中断服务函数
	static float TargetCurrent = 0;	// 临时目标电流
	static int8_t ret = 0;	// 返回值

	if (adc_interrupt_flag_get(ADC1, ADC_INT_FLAG_EOIC) == RESET)
	{/// 出现未知中断
		motor.CurMode = MM_Error;
		motor.ErrorInfo = Error_Unknown;
		ClosePWM( );	// 关闭PWM
		nvic_irq_disable(ADC0_1_IRQn); // 关闭ADC0_1中断
		return;
	}
	adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);	// 清除ADC中断标志

	gpio_bit_set(GPIOA, GPIO_PIN_1);	// 测试用

	getMotorCurrent(&motor);	// 获取电机U、V、W相电流
	if (motor.ErrorInfo != Error_EncoderReadError)
		if (Encoder_UpdateValue( ) < 0)
		{
			ClosePWM( );	// 关闭PWM
			motor.NextMode = MM_Error;
			motor.ErrorInfo = Error_EncoderReadError;
		}

	if (motor.CurMode != motor.NextMode)	// 电机模式需要发生改变
	{
		QuitMotorMode( );	// 退出当前电机模式
		EnterMotorMode( );	// 进入下一电机模式
		ret = 0;	// 重置返回值
	}
	switch (motor.CurMode)
	{
	case MM_NULL:
		break;

	case MM_DetectingResistance:	// 检测电机电阻
		ret = DetectingResistance(&motor, motor.MaxCurrent * _1_DIV_3, motor.Udc * _1_DIV_3, False);
		if (ret == 1)	// 电阻检测完成
			motor.NextMode = MM_NULL;
		else if (ret < 0)	// 电阻检测错误
			motor.NextMode = MM_Error;
		break;

	case MM_DetectingInductance:	// 检测电机电感
		ret = DetectingInductance(&motor, motor.MaxCurrent * motor.Resistance * _1_DIV_3, False);
		if (ret == 1)	// 电感检测完成
			motor.NextMode = MM_NULL;
		else if (ret < 0)	// 电感检测错误
			motor.NextMode = MM_Error;
		break;

	case MM_EncoderCalibration:	// 编码器校准
		ret = EncoderOffsetCalibration(&motor, False);
		if (ret == 1)	// 编码器校准完成
			motor.NextMode = MM_NULL;
		else if (ret < 0)	// 编码器校准错误
			motor.NextMode = MM_Error;
		break;

	case MM_OpenLoopVF:	// 开环VF控制
		motor.Ud = 0.0F;
		// motor.Uq = motor.MaxCurrent * motor.Resistance * _1_DIV_3;
		motor.Uq = 0.3;
		// 计算累积目标角度
		motor.OpenLoopTargetAngle_VF += motor.OpenLoopTargetSpeed * FOC_CONTROL_PERIOD * motor.PolePairs;
		motor.Angle = fmodf(motor.OpenLoopTargetAngle_VF, _2PI);	// 设置电机电角度
		fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); // 计算角度sin,cos值
		ApplyMotorInfo(&motor);	// 将电机信息应用到电机
		break;

	case MM_OpenLoopIF:	// 开环IF控制
		motor.TargetCurrent = 0.5F;
		motor.OpenLoopTargetAngle_IF += motor.OpenLoopTargetSpeed * FOC_CONTROL_PERIOD * motor.PolePairs;
		motor.Angle = fmodf(motor.OpenLoopTargetAngle_IF, _2PI);	// 限制在-2PI~2PI之间
		goto CurrentControl;

	case MM_AnticoggingCalibration:	// 抗齿槽力矩校准
		ret = AnticoggingCalibration(&motor, False);
		if (ret == 1) motor.NextMode = MM_NULL;
		else if (ret < 0) motor.NextMode = MM_Error;
	case MM_PositionControl:	// 位置环控制
		PIDSingleCalc(&motor.PIDInfoPosition, motor.TargetPosition, Encoder.AccAngle);
		motor.TargetSpeed = motor.PIDInfoPosition.output;	// 设置目标速度
		LIMIT(motor.TargetSpeed, -motor.MaxSpeed, motor.MaxSpeed);	// 限制在最大速度范围内
	case MM_SpeedControl:	// 速度环控制
		PIDSingleCalc(&motor.PIDInfoSpeed, motor.TargetSpeed, Encoder.Speed);
		motor.TargetCurrent = motor.PIDInfoSpeed.output;	// 设置目标电流
		LIMIT(motor.TargetCurrent, -motor.MaxCurrent, motor.MaxCurrent);	// 限制在最大电流范围内
	case MM_CurrentControl:	// 电流环控制
		motor.Angle = Encoder.Angle * motor.PolePairs;	// 设置电机电角度
		motor.Angle = fmodf(motor.Angle, _2PI);	// 限制在-2PI~2PI之间
CurrentControl:
		fast_sin_cos(motor.Angle, &motor.sinValue, &motor.cosValue); // 计算角度sin值和cos值
		Clarke_Transf(&motor);	// Clarke变换
		Park_Transf(&motor);	// Park变换
		if (motor.CurMode != MM_AnticoggingCalibration && motor.AnticoggingCalibratedFlag == 1)// 抗齿槽力矩补偿
			TargetCurrent = motor.TargetCurrent + motor.AnticogongTorqueTable[(uint32_t)roundf((float)Encoder.RawCount / ANTICOGING_INCREMENT)];
		else
			TargetCurrent = motor.TargetCurrent;
		PIDSingleCalc(&motor.PIDInfoIQ, TargetCurrent, motor.Iq);	// Iq控制
		motor.Uq = motor.PIDInfoIQ.output;	// 设置Uq
		PIDSingleCalc(&motor.PIDInfoID, 0.0F, motor.Id);	// Id控制
		motor.Ud = motor.PIDInfoID.output;	// 设置Ud
		ApplyMotorInfo(&motor);	// 将电机信息应用到电机
		break;

	case MM_Error:
	default:
		break;
	}

	gpio_bit_reset(GPIOA, GPIO_PIN_1);	// 测试用
}



