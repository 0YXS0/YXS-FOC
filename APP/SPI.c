#include "SPI.h"
#include "gd32f30x.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_spi.h"

#define TIMEOUT (0xFFFF)  //超时时间

/// @brief 初始化SPI0引脚
static void SPI0_GPIO_Config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE); // 关闭JTAG引脚功能
    gpio_pin_remap_config(GPIO_SPI0_REMAP, ENABLE); // SPI0引脚重映射
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);   // SPI0_SCK
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4);   // SPI0_MISO
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);   // SPI0_MOSI
}

/// @brief 初始化SPI0
void SPI0_Config(void)
{
    SPI0_GPIO_Config( );    // 初始化SPI0引脚
    rcu_periph_clock_enable(RCU_SPI0);  // 使能SPI0时钟

    spi_i2s_deinit(SPI0);
    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;  // 全双工
    spi_init_struct.device_mode = SPI_MASTER;   // 主机模式
    spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;   // 数据帧大小
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE; // 时钟极性和相位
    spi_init_struct.nss = SPI_NSS_SOFT;   // NSS信号(硬件控制)
    spi_init_struct.prescale = SPI_PSC_16;   // 预分频(120MHz / 16 = 7.5MHz)
    spi_init_struct.endian = SPI_ENDIAN_MSB;    // 数据帧传输顺序
    spi_init(SPI0, &spi_init_struct);
    spi_enable(SPI0);   // 使能SPI0
}

/// @brief SPI0读取数据(使用前需要先拉低片选信号)
/// @param data 读取数据
/// @return 0:成功；其他:失败
int8_t SPI0_ReadData(uint16_t* const data)
{
    uint32_t Count = 0;

    Count = 0;
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))
    {/// 等待接收缓冲区非空
        if (Count++ > TIMEOUT)
            return -1;
    }
    *data = spi_i2s_data_receive(SPI0); // 读取数据

    return 0;
}

/// @brief SPI0写入数据(使用前需要先拉低片选信号)
/// @param data 要写入的数据
/// @return 0:成功；其他:失败
int8_t SPI0_WriteData(const uint16_t data)
{
    uint32_t Count = 0;
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))
    {/// 等待发送缓冲区空
        if (Count++ > TIMEOUT)
            return -1;
    }

    // 发送数据
    spi_i2s_data_transmit(SPI0, data);

    Count = 0;
    while (spi_i2s_flag_get(SPI0, SPI_FLAG_TRANS))
    {/// 等待传输完成
        if (Count++ > TIMEOUT)
            return -2;
    }
    return 0;
}



