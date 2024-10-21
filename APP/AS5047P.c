#include "AS5047P.h"
#include "SPI.h"
#include "gd32f30x_gpio.h"

/*内容随外界变化的寄存器 可读不可写*/
#define NOP_ADDR      0x0000    // 启动读取过程寄存器地址
#define ERRFL_ADDR    0x0001    // 错误寄存器地址
#define PROG_ADDR     0x0003    // 编程寄存器地址
#define DIAAGC_ADDR   0x3FFC    // 诊断和AGC寄存器地址
#define MAG_ADDR      0x3FFD    // CORDIC寄存器地址
#define ANGLEUNC_ADDR 0x3FFE    // 无动态角度误差补偿的测量角度寄存器地址
#define ANGLECOM_ADDR 0x3FFF    // 带动态角度误差补偿的测量角度寄存器地址

/*配置选项寄存器 可读可写*/
#define ZPOSM         0x0016
#define ZPOSL         0x0017
#define SETTINGS1     0x0018
#define SETTINGS2     0x0019

/*
AS50047P拥有三种帧格式，分别为命令帧、读取帧、写入帧
命令帧格式: bit15:偶校验位 bit14:R(1) W(0)  bit13-0为寄存器地址 该帧由主机发送给AS5047P
读取帧格式: bit15:偶校验位 bit14:前一次指令错误(1)  bit13-0为读取的数据 该帧由AS5047P返回给主机
写入帧格式: bit15:偶校验位 bit14:固定为0    bit13-0为写入的数据 该帧由主机发送给AS5047P
*/
/// @brief 读取寄存器命令
#define COM_READ_ANGLECOM 0xFFFF    // 读取带动态角度误差补偿的测量角度寄存器命令
#define COM_READ_NOP      0xC000    // 空操作寄存器命令
#define COM_READ_ERRFL    0x4001    // 错误标志寄存器命令

/// @brief AS5047P初始化
void AS5047P_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE); // 关闭JTAG引脚功能
    gpio_init(GPIOA, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_15); // 初始化CS引脚
}

/// @brief AS5047P读取计数值
/// @param data 数据
/// @return 0:成功；其他:失败
int8_t AS5047P_ReadCount(uint16_t* const data)
{
    uint8_t count = 0;
    int ret = 0;

    gpio_bit_reset(GPIOA, GPIO_PIN_15); // 片选信号拉低
    ret = SPI0_WriteData(COM_READ_ANGLECOM); // 写入读取带动态角度误差补偿的测量角度寄存器命令
    if (ret != 0)
    {/// 写入失败
        gpio_bit_set(GPIOA, GPIO_PIN_15); // 片选信号拉高
        *data = 0;
        return ret;
    }
    ret = SPI0_ReadData(data); // 读取数据
    if (ret != 0)
    {/// 读取失败
        gpio_bit_set(GPIOA, GPIO_PIN_15); // 片选信号拉高
        *data = 0;
        return ret;
    }
    gpio_bit_set(GPIOA, GPIO_PIN_15); // 片选信号拉高

    // 检查错误
    if (*data & (1 << 14))
    {
        //有错误 发送清除错误指令
        (void)SPI0_WriteData(COM_READ_ERRFL);
        return -10;
    }
    // 偶校验
    for (uint8_t i = 0; i < 16; i++)
    {
        if (*data & (0x01 << i))
            count++;
    }
    if (count & 0x01)
    {// 偶校验错误
        *data = 0;
        return -1;
    }

    *data &= 0x3FFF; // 保留低14位
    return ret;
}

