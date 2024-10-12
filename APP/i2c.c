#include "gd32f30x.h"
#include "i2c.h"
#include <stdio.h>

#define TIMEOUT (uint32_t)0x0FFF  //超时时间

/// @brief I2C GPIO Init
void i2c0_gpio_config(void)
{
    rcu_periph_clock_enable(I2C0_GPIO_CLOCK);

    gpio_init(I2C0_SCL_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C0_SCL_PIN); //GPIO_PIN_6
    gpio_init(I2C0_SDA_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C0_SDA_PIN); //GPIO_PIN_7
}

/// @brief i2c Init
void i2c0_config(void)
{
    i2c0_gpio_config( );
    rcu_periph_clock_enable(RCU_I2C0);  //开启I2C0时钟
    rcu_periph_clock_enable(RCU_AF);    //开启AF时钟
    i2c_clock_config(I2C0, I2C0_SPEED, I2C_DTCY_16_9);  //设置I2C0时钟频率
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);   //设置I2C0为I2C模式，7位地址

    i2c_enable(I2C0);   //使能I2C0
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);   //使能应答
}

#if 0

static uint8_t I2C0_Status = 0; //I2C0状态 0:发数据 1:读数据
static uint8_t I2C0_DecAdder = 0xFF;    //从机地址
static uint8_t* I2C0_RXData = NULL;    //I2C0接收数据存放地址
static uint8_t I2C0_RXDataSize = 0;    //I2C0接收数据长度

/// @brief I2C0读数据(中断方式)
/// @param DevAddress 从机地址
/// @param data 数据存放地址
/// @param dataSize 数据长度
/// @return 0:成功；其他:失败
int8_t I2C0_Read(uint8_t DevAddress, uint8_t* data, uint8_t dataSize)
{
    if (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
    {
        i2c_stop_on_bus(I2C0); //发送停止信号
        return -1;
    }

    I2C0_DecAdder = DevAddress;   //从机地址
    I2C0_RXData = data;  //接收数据存放地址
    I2C0_RXDataSize = dataSize; //接收数据长度

    i2c_ack_config(I2C0, I2C_ACK_ENABLE);   //使能应答
    i2c_start_on_bus(I2C0); //发送起始信号
    return 0;
}

/// @brief i2c0中断处理函数
void I2C0_EV_IRQHandler(void)
{
    if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_SBSEND))
    {
        if (I2C0_Status == 0)
            i2c_master_addressing(I2C0, I2C0_DecAdder, I2C_TRANSMITTER); //发送模式,发送从机地址
        else if (I2C0_Status == 1)
            i2c_master_addressing(I2C0, I2C0_DecAdder, I2C_RECEIVER); //接收模式,发送从机地址
    }
    else if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_ADDSEND))
    {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_ADDSEND);

        if (I2C0_Status == 0)
        {
            i2c_data_transmit(I2C0, 0x0C); //发送寄存器地址
        }
    }
    else if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_TBE))
    {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_TBE);

        if (I2C0_Status == 0)
        {
            I2C0_Status = 1;    //进入接收模式
            i2c_start_on_bus(I2C0); //发送起始信号
        }
    }
    else if (i2c_interrupt_flag_get(I2C0, I2C_INT_FLAG_RBNE))
    {
        i2c_interrupt_flag_clear(I2C0, I2C_INT_FLAG_RBNE);
        // 最后一个数据不应答
        if (--I2C0_RXDataSize == 1) i2c_ack_config(I2C0, I2C_ACK_DISABLE);  //关闭应答
        *I2C0_RXData++ = i2c_data_receive(I2C0);    //读取数据
        if (I2C0_RXDataSize == 0)
        {
            i2c_stop_on_bus(I2C0); //发送停止信号
            //i2c_ack_config(I2C0, I2C_ACK_ENABLE);   //使能应答
            I2C0_Status = 0;    //进入发送模式
        }
    }
}
#endif

/// @brief i2c写数据
/// @param I2CX i2c设备
/// @param DevAddress  从机地址
/// @param data    要写入的数据
/// @param dataSize  写入的数据长度
/// @return 0:成功；其他:失败
int8_t i2c_write(uint32_t I2CX, uint8_t DevAddress, uint8_t RegAddress, const uint8_t* data, uint8_t dataSize)
{
    uint32_t Timeout_t = 0;
    uint8_t i = 0;
    int8_t ret = 0;

    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2CX, I2C_FLAG_I2CBSY))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -1;
            goto end;
        }
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2CX);

    Timeout_t = TIMEOUT;
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2CX, I2C_FLAG_SBSEND))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -2;
            goto end;
        }
    }
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2CX, DevAddress, I2C_TRANSMITTER);

    Timeout_t = TIMEOUT;
    /* wait until ADDSEND bit is set*/
    while (!i2c_flag_get(I2CX, I2C_FLAG_ADDSEND))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -3;
            goto end;
        }
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2CX, I2C_FLAG_ADDSEND);

    // send regaddres
    i2c_data_transmit(I2CX, RegAddress);
    Timeout_t = TIMEOUT;
    /* wait until the transmission data register is empty*/
    while (!i2c_flag_get(I2CX, I2C_FLAG_TBE))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -4;
            goto end;
        }
    }

    // send data
    for (i = 0;i < dataSize;i++)
    {
        /* send a data byte */
        i2c_data_transmit(I2CX, data[i]);

        Timeout_t = TIMEOUT;
        /* wait until the transmission data register is empty*/
        while (!i2c_flag_get(I2CX, I2C_FLAG_TBE))
        {
            if (Timeout_t > 0)	Timeout_t--;
            else
            {
                ret = -5;
                goto end;
            }
        }
    }
end:
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2CX);

    Timeout_t = TIMEOUT;
    /* wait until stop condition generate */
    while (I2C_CTL0(I2CX) & 0x0200)
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            if (ret < 0) return ret;
            ret = -6;
            break;
        }
    }

    return ret;
}


/// @brief i2c读数据
/// @param I2CX i2c设备
/// @param DevAddress  从机地址
/// @param RegAddress  寄存器地址
/// @param data     读取的数据
/// @param dataSize 读取的数据长度
/// @return 0:成功；其他:失败
int8_t i2c_read(uint32_t I2CX, uint8_t DevAddress, uint8_t RegAddress, uint8_t* data, uint8_t dataSize)
{
    uint32_t Timeout_t = 0;
    uint8_t i = 0;
    int8_t ret = 0;

    /******************************************************/
    /*	Send Slave address and Specified Register Address */
    /******************************************************/
    Timeout_t = TIMEOUT;
    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2CX, I2C_FLAG_I2CBSY))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -1;
            goto end;
        }
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2CX);

    Timeout_t = TIMEOUT;
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2CX, I2C_FLAG_SBSEND))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -2;
            goto end;
        }
    }
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2CX, DevAddress, I2C_TRANSMITTER);//I2C_RECEIVER		I2C_TRANSMITTER

    Timeout_t = TIMEOUT;
    /* wait until ADDSEND bit is set*/
    while (!i2c_flag_get(I2CX, I2C_FLAG_ADDSEND))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -3;
            goto end;
        }
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2CX, I2C_FLAG_ADDSEND);

    /* send regaddres */
    i2c_data_transmit(I2CX, RegAddress);

    Timeout_t = TIMEOUT;
    /* wait until the transmission data register is empty*/
    while (!i2c_flag_get(I2CX, I2C_FLAG_TBE))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -4;
            goto end;
        }
    }
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2CX);

    Timeout_t = TIMEOUT;
    /* wait until stop condition generate */
    while (I2C_CTL0(I2CX) & 0x0200)
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -5;
            goto end;
        }
    }
    /* enable acknowledge */
    i2c_ack_config(I2CX, I2C_ACK_ENABLE);

    /******************************************************/
    /*	    Send Slave address and Read Data 	 		  */
    /******************************************************/

    Timeout_t = TIMEOUT;
    /* wait until I2C bus is idle */
    while (i2c_flag_get(I2CX, I2C_FLAG_I2CBSY))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -6;
            goto end;
        }
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2CX);

    Timeout_t = TIMEOUT;
    /* wait until SBSEND bit is set */
    while (!i2c_flag_get(I2CX, I2C_FLAG_SBSEND))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -7;
            goto end;
        }
    }

    /* send slave address to I2C bus*/
    i2c_master_addressing(I2CX, DevAddress, I2C_RECEIVER);

    Timeout_t = TIMEOUT;
    /* wait until ADDSEND bit is set*/
    while (!i2c_flag_get(I2CX, I2C_FLAG_ADDSEND))
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -8;
            goto end;
        }
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(I2CX, I2C_FLAG_ADDSEND);

    for (i = 0;i < dataSize;i++)
    {
        Timeout_t = TIMEOUT;
        /* wait until the RBNE bit is set */
        while (!i2c_flag_get(I2CX, I2C_FLAG_RBNE))
        {
            if (Timeout_t > 0)	Timeout_t--;
            else
            {
                ret = -9;
                goto end;
            }
        }
        /* read data from I2C_DATA */
        data[i] = i2c_data_receive(I2CX);

        //最后一个数据不应答
        if (i == dataSize - 2)
        {
            /* disable acknowledge */
            i2c_ack_config(I2CX, I2C_ACK_DISABLE);
        }
    }
end:
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2CX);

    Timeout_t = TIMEOUT;
    /* wait until stop condition generate */
    while (I2C_CTL0(I2CX) & 0x0200)
    {
        if (Timeout_t > 0)	Timeout_t--;
        else
        {
            ret = -10;
            break;
        }
    }

    /* enable acknowledge */
    i2c_ack_config(I2CX, I2C_ACK_ENABLE);
    return ret;
}

