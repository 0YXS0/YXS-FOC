#include "AS5600.h"
#include "i2c.h"
#include "gd32f30x.h"

#define AS5600_I2C I2C0
#define AS5600_ADDR 0x36 << 1   //I2C地址
#define Data_Hight_Register_Addr 0x0C   //寄存器高位地址
#define Data_Low_Register_Addr   0x0D   //寄存器低位地址

/// @brief I2C读取数据
/// @param RegAddress 寄存器地址
/// @param Data 数据
/// @param size 数据长度
/// @return 成功返回0，失败返回负数
static inline int8_t  AS5600_I2C_RecvData(uint8_t* Data, const uint8_t size)
{
    return i2c_read(AS5600_I2C, AS5600_ADDR, Data_Hight_Register_Addr, Data, size);
}

/// @brief AS5600读取原始角度(顺时针为正，逆时针为负)
/// @return 编码器原始计数
uint16_t AS5600_GetCount(void)
{
    static uint8_t data[2] = { 0,0 };
    AS5600_I2C_RecvData(data, 2);
    // return (data[0] << 8 | data[1]) * 0.001534F;
    return (data[0] << 8 | data[1]);
}





