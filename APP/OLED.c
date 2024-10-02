#include "gd32f30x.h"
#include "gd32f30x_i2c.h"

#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"

#define OLED_ADDRESS 0x78
#define OLED_I2C I2C0

/// @brief OLED写命令
/// @param Command 命令
/// @param size 命令数量
static inline int8_t OLED_WriteCommand(const uint8_t* Command, uint8_t size)
{
	return I2C_SendData(OLED_I2C, OLED_ADDRESS, 0x00, Command, size);
}

/// @brief OLED写数据
/// @param Data 数据
/// @param size 数据大小
static inline int8_t OLED_WriteData(const uint8_t* Data, uint8_t size)
{
	return I2C_SendData(OLED_I2C, OLED_ADDRESS, 0x40, Data, size);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0-7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0-127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	uint8_t Command[3] = { 0 };
	Command[0] = 0xB0 | Y;	//设置Y位置
	Command[1] = 0x10 | ((X & 0xF0) >> 4);	//设置X位置低4位
	Command[2] = 0x00 | (X & 0x0F);	//设置X位置高4位
	OLED_WriteCommand(Command, 3);
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{
	uint8_t  i;
	uint8_t Command[128] = { 0 };
	for (i = 0; i < 8; i++)
	{
		OLED_SetCursor(i, 0);
		OLED_WriteData(Command, 128);
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1-4
  * @param  Column 列位置，范围：1-16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	const uint8_t* data = OLED_F8x16[Char - ' '];
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	OLED_WriteData(data, 8);
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	OLED_WriteData(data + 8, 8);
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1-4
  * @param  Column 起始列位置，范围：1-16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char* String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1-4
  * @param  Column 起始列位置，范围：1-16
  * @param  Number 要显示的数字，范围：0-4294967295
  * @param  Length 要显示数字的长度，范围：1-10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1-4
  * @param  Column 起始列位置，范围：1-16
  * @param  Number 要显示的数字，范围：-2147483648-2147483647
  * @param  Length 要显示数字的长度，范围：1-10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1-4
  * @param  Column 起始列位置，范围：1-16
  * @param  Number 要显示的数字，范围：0-0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1-8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1-4
  * @param  Column 起始列位置，范围：1-16
  * @param  Number 要显示的数字，范围：0-1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1-16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	uint16_t i, j;
	uint8_t Command[23] = { 0 };

	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}

	Command[0] = 0xAE;	//关闭显示
	Command[1] = 0xD5;	//设置显示时钟分频比/振荡器频率
	Command[2] = 0x80;
	Command[3] = 0xA8;	//设置多路复用率
	Command[4] = 0x3F;
	Command[5] = 0xD3;	//设置显示偏移
	Command[6] = 0x00;
	Command[7] = 0x40;	//设置显示开始行
	Command[8] = 0xA1;	//设置左右方向，0xA1正常 0xA0左右反置
	Command[9] = 0xC8;	//设置上下方向，0xC8正常 0xC0上下反置
	Command[10] = 0xDA;	//设置COM引脚硬件配置
	Command[11] = 0x12;
	Command[12] = 0x81;	//设置对比度控制
	Command[13] = 0xCF;
	Command[14] = 0xD9;	//设置预充电周期
	Command[15] = 0xF1;
	Command[16] = 0xDB;	//设置VCOMH取消选择级别
	Command[17] = 0x30;
	Command[18] = 0xA4;	//设置整个显示打开/关闭
	Command[19] = 0xA6;	//设置正常/倒转显示
	Command[20] = 0x8D;	//设置充电泵
	Command[21] = 0x14;
	Command[22] = 0xAF;	//开启显示
	OLED_WriteCommand(Command, 23);

	OLED_Clear();				//OLED清屏
}
