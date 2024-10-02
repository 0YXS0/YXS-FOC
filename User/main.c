#include "gd32f30x.h"
#include "systick.h"
#include "Delay.h"
#include "i2c.h"
#include "OLED.h"
#include "Encoder.h"
#include "ADC.h"
#include "Timer.h"
#include "Filter.h"
#include "Motor.h"
#include "PID.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"

#define _PI 3.1415927F
#define _2PI 6.2831853F	//2PI
#define _4PI_5 2.5132741F	//4PI/5
#define _3PI_2 4.7123889F	//3PI/2
#define _PI_10 0.3141593F	//PI/10
#define _19PI_10 5.9690260F	//19PI/10
#define _PI_2 1.5707963F	//PI/2
#define _PI_4 0.7853982F	//PI/4

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))	//限幅
#define MAX(a,b) ((a)>(b)?(a):(b))	//求最大值
#define MIN(a,b) ((a)<(b)?(a):(b))	//求最小值
#define ABS(x) ((x)>0?(x):-(x))	//求绝对值
#define ReductionRatio 15	//减速器减速比

// static float Offset_Angle = 0;	//电角度零点
// static uint8_t MotorStatus = 0;	//电机状态

// I2C_Config_Info I2C0_Config_Info = {
// 	.I2C_id = I2C0,
// 	.Address = 0xA0,
// 	.AddFormat = I2C_ADDFORMAT_7BITS,
// 	.Speed = 400000U,
// 	.Clock_I2C = RCU_I2C0,
// 	.Clock_SCL = RCU_GPIOB,
// 	.Clock_SDA = RCU_GPIOB,
// 	.Port_SCL = GPIOB,
// 	.Port_SDA = GPIOB,
// 	.Pin_SCL = GPIO_PIN_6,
// 	.Pin_SDA = GPIO_PIN_7,
// };	//I2C0配置信息

// KalmanFilterInfo FilterInfo_U = { 10, 0.001, 0.001, 0, 0 };	//电机U相电流卡尔曼滤波器参数
// KalmanFilterInfo FilterInfo_V = { 10, 0.001, 0.001, 0, 0 };	//电机V相电流卡尔曼滤波器参数
// KalmanFilterInfo FilterInfo_W = { 10, 0.001, 0.001, 0, 0 };	//电机W相电流卡尔曼滤波器参数
// KalmanFilterInfo FilterInfo_Power = { 10, 0.001, 0.001, 0, 0 };	//电源电压卡尔曼滤波器参数
//KalmanFilterInfo FilterInfo_Angle = { 10, 0.001, 0.001, 0, 0 };	//角度卡尔曼滤波器参数
FirstOrderFilterInfo FilterInfo_U = { 0.8F,0.2F, 0 };	//电机U相电流一阶互补滤波器参数
// FirstOrderFilterInfo FilterInfo_V = { 0.8F,0.2F, 0 };	//电机V相电流一阶互补滤波器参数
FirstOrderFilterInfo FilterInfo_W = { 0.8F,0.2F, 0 };	//电机W相电流一阶互补滤波器参数

// 获取电机U相电流滤波后的值
static inline float get_U_Value(void) { return FirstOrderFilter(&FilterInfo_U, adc_getRawValue(_U_Value)); }
// 获取电机V相电流滤波后的值
// static inline float get_V_Value(void) { return FirstOrderFilter(&FilterInfo_V, adc_getRawValue(_V_Value)); }
// 获取电机W相电流滤波后的值
static inline float get_W_Value(void) { return FirstOrderFilter(&FilterInfo_W, adc_getRawValue(_W_Value)); }

static MotorInfo motor = {
	.MotorID = 0x01,
	.PolePairs = 7,
	.MAXPulse = 1000,
	.Udc = 12.0F,
	.Uq = 0.0F,
	.Ud = 0.0F,
};

static inline void setPWM(MotorInfo* motor)	//设置PWM
{
	// //设置占空比
	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, motor->PulseA);
	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, motor->PulseB);
	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, motor->PulseC);
	timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1,
		MAX(MAX(motor->PulseA, motor->PulseB), motor->PulseC) + 15);
}

// //电机校准-找电角度零点
// void MotorCalibration(void)
// {
// 	motor.Uq = 0.0F;	//电机电流
// 	motor.Ud = 1.0F;
// 	motor.Angle = 0.0F;	//电机角度
// 	Rev_Park_Transf(&motor);	//逆Park变换
// 	SVPWM(&motor);	//SVPWM
// 	setPWM(&motor);	//设置PWM
// 	Delay_ms(1000);
// 	// AS5600_GetRawAngle( );	//获取电机角度
// 	// Delay_ms(10);
// 	// Offset_Angle = AS5600_GetRawAngle( );	//获取电机角度
// 	Delay_ms(10);
// 	motor.Uq = 0.0F;	//电机电流
// 	motor.Ud = 0.0F;	//电机电流
// 	motor.PulseA = 0;
// 	motor.PulseB = 0;
// 	motor.PulseC = 0;
// 	setPWM(&motor);	//设置PWM
// 	timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 125);   //占空比
// }

float lastTime = 0, curTime = 0, time = 0;
int main( )
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);   // 设置中断分组
	systick_config( );
	i2c0_config( );	//I2C0配置
	usart_config( );	//串口配置
	adc_config( );	//ADC0配置，对U、V、W相电流和电源电压进行采样
	timer_config(6, 1000);	//PWM频率(中央对齐模式,频率减半)--(120MHz / 2 / 2 / 1000 = 30kHz)(电流环频率 = 30kHz / 3 = 10kHz)
	Delay_ms(100);	//延时100ms

	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	adc_OffsetConfig( );	//初始化电机U、W相电流和电源电压偏置
	nvic_irq_enable(ADC0_1_IRQn, 1, 1); // 使能ADC0_1中断
	Delay_ms(10);	//延时10ms
	// MotorStatus = 1;	//电机开启
	while (1)
	{
		JustFloat_Show(6, get_U_Value( ), get_W_Value( ), adc_getRawValue(_Power_Value), Encoder_GetValue(EVT_Angle), Encoder_GetValue(EVT_AccAngle), Encoder_GetValue(EVT_Speed));
	}
}

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
	setPWM(motor);	//设置PWM
}

void SpeedOpenloop(float targetSpeed)
{
	static float targetAngle = 0;

	motor.Uq = 0.6F;
	//计算累积目标角度
	targetAngle += targetSpeed * 0.0001F * motor.PolePairs;
	//targetAngle = (get_Angle_Value( ) + targetSpeed * 0.0001F) * motor.PolePairs;	//获取电机角度
	//targetAngle = get_Angle_Value( ) * motor.PolePairs;	//获取电机角度
	motor.Angle = targetAngle;	//设置电机角度
	//motor.Angle = fmodf(targetAngle, _2PI);	//限制在-2PI~2PI之间
	//motor.Angle = motor.Angle >= 0.0F ? motor.Angle : motor.Angle + _2PI;//限制在0~2PI之间

	Rev_Park_Transf(&motor);	//逆Park变换
	//Rev_Clarke_Transf(&motor);	//逆Clarke变换
	SVPWM(&motor);	//SVPWM
	setPWM(&motor);	//设置PWM
}

//ADC0中断服务函数
void ADC0_1_IRQHandler(void)
{
	if (adc_interrupt_flag_get(ADC1, ADC_INT_FLAG_EOIC) != RESET)
	{// ADC转换完成,进行一次Foc控制
		adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);	//清除ADC中断标志

		gpio_bit_set(GPIOA, GPIO_PIN_1);	//测试用
		Encoder_UpdateValue( );	//更新编码器值
		// if (MotorStatus == 0) return;//电机关闭

		static uint8_t count = 0;
		// static float angle = 0, lastAngle = 0, speed = 0;
		// angle = get_Angle_Value( );	//获取电机角度
		//sum += angle - lastAngle;	//累积角度
		// if (count == 10)
		// {
		// 	speed = FirstOrderFilter(&FilterInfo_Speed, (angle - lastAngle) * 1000.0F);	//计算电机速度
		// 	//speed = (angle - lastAngle) / 0.001F;	//计算电机速度
		// 	PID_SingleCalc(&PIDInfo_Speed, target, speed);	//速度环控制
		// 	lastAngle = angle;
		// }

		//setPWM(&motor);	//设置PWM
		// SpeedOpenloop(120.0F);
		// CurrentControl(&motor, PIDInfo_Speed.output, angle);	//电流环控制
		if (count++ == 10) count = 0;
		gpio_bit_reset(GPIOA, GPIO_PIN_1);	//测试用
	}
}

/*
void TIMER1_IRQHandler(void)
{
	if (timer_interrupt_flag_get(TIMER1, TIMER_INT_UP) != RESET)
	{
		timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);	//清除更新中断标志

		static uint8_t count = 0;
		static float angle = 0, lastAngle = 0, speed = 0;
		angle = get_Angle_Value( );	//获取电机角度
		//sum += angle - lastAngle;	//累积角度
		if (count == 10)
		{
			speed = FirstOrderFilter(&FilterInfo_Speed, (angle - lastAngle) * 1000.0F);	//计算电机速度
			//speed = (angle - lastAngle) / 0.001F;	//计算电机速度
			PID_SingleCalc(&PIDInfo_Speed, target, speed);	//速度环控制
			lastAngle = angle;
		}
		if (MotorStatus == 0) return;//电机关闭

		//setPWM(&motor);	//设置PWM
		SpeedOpenloop(120.0F);
		//CurrentControl(&motor, PIDInfo_Speed.output, angle);	//电流环控制
		JustFloat_Show(9, get_U_Value( ), get_V_Value( ), get_W_Value( ), angle, PIDInfo_Speed.output, target, motor.Iq, motor.Id, speed);
		if (count++ == 10) count = 0;
	}
}*/



