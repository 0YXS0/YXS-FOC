#include "USART.h"
#include "stdio.h"
#include "stdarg.h"
#include "gd32f30x.h"
#include "stdlib.h"
#include "Command.h"


#define USART1_RX_SIZE  128
#define USART1_TX_SIZE  128
char USART1RX_Buffer[USART1_RX_SIZE] = { 0 };    //串口1接收缓存
// uint8_t USART1TX_Buffer[USART1_TX_SIZE] = { 0 };    //串口1发送缓存

/// @brief 串口1 DMA配置
void usart_dma_config(void)
{
    dma_parameter_struct dma_init_Usart1_TX;
    dma_parameter_struct dma_init_Usart1_RX;

    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);      //使能DMA0时钟

    dma_deinit(DMA0, DMA_CH5);  //复位DMA0通道5
    dma_deinit(DMA0, DMA_CH6);  //复位DMA0通道6

    //配置DMA0通道5(Usart1_RX)
    dma_init_Usart1_RX.direction = DMA_PERIPHERAL_TO_MEMORY;        //方向:外设到内存
    dma_init_Usart1_RX.memory_addr = (uint32_t)USART1RX_Buffer;     //内存地址
    dma_init_Usart1_RX.memory_inc = DMA_MEMORY_INCREASE_ENABLE;     //内存地址自增
    dma_init_Usart1_RX.memory_width = DMA_MEMORY_WIDTH_8BIT;        //内存数据宽度
    dma_init_Usart1_RX.number = (uint32_t)USART1_RX_SIZE;           //数据传输量
    dma_init_Usart1_RX.periph_addr = (uint32_t)(&USART_DATA(USART1));   //外设地址
    dma_init_Usart1_RX.periph_inc = DMA_PERIPH_INCREASE_DISABLE;        //外设地址不自增
    dma_init_Usart1_RX.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;        //外设数据宽度
    dma_init_Usart1_RX.priority = DMA_PRIORITY_LOW; //优先级为低
    dma_init(DMA0, DMA_CH5, &dma_init_Usart1_RX);
    dma_circulation_disable(DMA0, DMA_CH5); //禁止循环传输

    //配置DMA0通道6(Usart1_TX)
    dma_init_Usart1_TX.direction = DMA_MEMORY_TO_PERIPHERAL;        //方向:内存到外设
    dma_init_Usart1_TX.memory_addr = (uint32_t)NULL;     //内存地址
    dma_init_Usart1_TX.memory_inc = DMA_MEMORY_INCREASE_ENABLE;     //内存地址自增
    dma_init_Usart1_TX.memory_width = DMA_MEMORY_WIDTH_8BIT;        //内存数据宽度
    dma_init_Usart1_TX.number = (uint32_t)0;           //数据传输量
    dma_init_Usart1_TX.periph_addr = (uint32_t)(&USART_DATA(USART1));   //外设地址
    dma_init_Usart1_TX.periph_inc = DMA_PERIPH_INCREASE_DISABLE;        //外设地址不自增
    dma_init_Usart1_TX.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;        //外设数据宽度
    dma_init_Usart1_TX.priority = DMA_PRIORITY_LOW; //优先级为低
    dma_init(DMA0, DMA_CH6, &dma_init_Usart1_TX);
    dma_circulation_disable(DMA0, DMA_CH6); //禁止循环传输

    dma_memory_to_memory_disable(DMA0, DMA_CH5);    //禁止内存到内存传输
    dma_memory_to_memory_disable(DMA0, DMA_CH6);    //禁止内存到内存传输

    dma_channel_enable(DMA0, DMA_CH5);  //使能DMA0通道5
    dma_channel_disable(DMA0, DMA_CH6); //使能DMA0通道6
}

/// @brief 串口1配置
void usart_config(void)
{
    // GPIO时钟使能
    rcu_periph_clock_enable(RCU_GPIOA);
    // USART时钟使能
    rcu_periph_clock_enable(RCU_USART1);

    // 配置TX为推挽复用模式
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    // 配置RX为浮空输入模式
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    // 配置串口的工作参数
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 1152000U);    // 波特率
    usart_word_length_set(USART1, USART_WL_8BIT);   // 帧数据字长
    usart_stop_bit_set(USART1, USART_STB_1BIT); // 停止位
    usart_parity_config(USART1, USART_PM_NONE); // 奇偶校验位
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);  // 硬件流控制RTS
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);  // 硬件流控制CTS
    usart_receive_config(USART1, USART_RECEIVE_ENABLE); // 使能接收
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);   // 使能发送  

    usart_dma_transmit_config(USART1, USART_TRANSMIT_DMA_ENABLE);    // 使能USART1的DMA接收
    usart_dma_receive_config(USART1, USART_RECEIVE_DMA_ENABLE);    // 使能USART1的DMA发送

    nvic_irq_enable(USART1_IRQn, 1, 1); //使能USART1中断
    usart_interrupt_enable(USART1, USART_INT_IDLE);    //使能USART1空闲中断

    usart_dma_config( ); // 配置串口1 DMA

    usart_enable(USART1);   // 使能串口
}

/// @brief 串口1 中断处理函数
void USART1_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE) == SET)
    {
        usart_data_receive(USART1);     //清USART_INT_FLAG_IDLE标志位
        dma_channel_disable(DMA0, DMA_CH5); //关闭DMA0通道5 
        dma_flag_clear(DMA0, DMA_CH5, DMA_INTF_FTFIF);  //清DMA0通道5标志位

        UsartCommandAnalyze(USART1RX_Buffer);    //串口命令解析

        dma_transfer_number_config(DMA0, DMA_CH5, USART1_RX_SIZE);      //重新设置数据传输量
        dma_channel_enable(DMA0, DMA_CH5);      //使能DMA0通道5
    }
}

/// @brief 串口1 DMA发送
/// @param data 数据指针
/// @param dataSize 数据大小
void usart1_dma_send(uint8_t* data, uint8_t dataSize)
{
    while (dma_transfer_number_get(DMA0, DMA_CH6) != 0);    //等待DMA通道6传输完成

    dma_channel_disable(DMA0, DMA_CH6);

    dma_memory_address_config(DMA0, DMA_CH6, (uint32_t)data);
    dma_transfer_number_config(DMA0, DMA_CH6, dataSize);

    /* enable DMA channel to start send */
    dma_channel_enable(DMA0, DMA_CH6);
}

#if 0

/// @brief 重定向c库函数printf到USART1
int fputc(int ch, FILE* f)
{
    usart1_send_data((uint8_t*)&ch, 1);
    return ch;
}

//配合浮点数转化使用

//将浮点数转化为4个字节数据存放在byte[4]中
void Float_to_Byte(float f, unsigned char byte [ ])
{
    FloatLongType fl;
    fl.fdata = f;
    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}

#endif

typedef union
{
    float fdata;
    uint32_t ldata;
}FloatLongType;

/// @brief 打印调试信息函数(配合VOFA+使用)
/// @param Num 可变参数个数
/// @param ... 可变参数(float类型)
void JustFloat_Show(uint8_t Num, ...)	//Justfloat 数据协议
{
    va_list args;
    va_start(args, Num);
    static uint8_t data[256];
    static FloatLongType fl;
    static uint8_t t = 0;

    for (uint8_t i = 0; i < Num; i++)	//发送数据
    {
        t = i * 4;
        fl.fdata = va_arg(args, double);
        data[t] = (uint8_t)fl.ldata;
        data[t + 1] = (uint8_t)(fl.ldata >> 8);
        data[t + 2] = (uint8_t)(fl.ldata >> 16);
        data[t + 3] = (uint8_t)(fl.ldata >> 24);
    }
    va_end(args);
    //发送帧尾
    t = Num * 4;
    data[t] = 0x00;
    data[t + 1] = 0x00;
    data[t + 2] = 0x80;
    data[t + 3] = 0x7f;

    usart1_dma_send(data, (Num + 1) * 4);    //dma发送数据
}
