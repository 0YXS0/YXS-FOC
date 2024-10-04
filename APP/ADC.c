#include "ADC.h"
#include "gd32f30x.h"
#include "gd32f30x_adc.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_dma.h"

#include "Delay.h"

#define GAIN 20.0F  // 电流采样芯片放大倍数
#define REFERENCE 1.2F  // 芯片内部参考电压1.2V
#define RESISTANCE 0.005F   // 采样电阻5mΩ
static const float Gain = GAIN * RESISTANCE;  // 电流采样电压增益
static uint16_t Offset_U = 0, Offset_W = 0;//电机U、V、W相电流偏置
static float Reference_Gain = 0;	//电源电压,单片机内部参考电压,内部参考电压增益

#define VALUE_NUM 2 // 采样值数量
static uint16_t adcRawValue[VALUE_NUM] = { 0, 0 };
static const uint16_t* Power_ADC_Value = adcRawValue + 0;   // 电源电压采样值
static const uint16_t* Reference_ADC_Value = adcRawValue + 1;   //内部参考电压采样值

/// @brief ADC_DMA配置
/// @param adc_value ADC采样值存放地址
void adc_dma_config(const uint16_t* adc_value)
{
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);

    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0)); // 外设寄存器地址
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;   // 外设地址不增加
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;  // 外设数据宽度
    dma_data_parameter.memory_addr = (uint32_t)(adc_value);        // 内存数据存放地址
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;    // 内存地址增加
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;      // 内存数据宽度
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;       // 方向:外设到内存
    dma_data_parameter.number = VALUE_NUM;                         // 数据传输量
    dma_data_parameter.priority = DMA_PRIORITY_HIGH;               // 优先级
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);// 初始化DMA
    dma_circulation_enable(DMA0, DMA_CH0);  // 循环传输

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

/// @brief ADC0进行电源电、内部参考电压初始化配置
void adc0_config(void)
{
    adc_dma_config(adcRawValue);    // ADC_DMA配置
    /*------------------时钟配置------------------*/
    // GPIO时钟使能
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);    // 使能AF时钟
    // ADC时钟使能
    rcu_periph_clock_enable(RCU_ADC0);
    // ADC时钟12分频，最大40MHz
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);

    /*------------------ADC GPIO配置------------------*/
    // 必须为模拟输入
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5); // 电源电压

    adc_deinit(ADC0);
    /*------------------ADC工作模式配置------------------*/
    // 独立模式
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);   // 使能扫描模式
    adc_tempsensor_vrefint_enable( );// 使能内部温度传感器和内部参考电压

    // 结果转换右对齐
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    // 转换通道数量
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 2);

    // 配置ADC通道转换顺序，采样时间为55.5个时钟周期
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5);    // 电源电压
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5);   // 单片机内部参考电压

    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);   // 软件触发
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE); // 规则组使能

    adc_dma_mode_enable(ADC0);  // 使能DMA模式
    // 使能ADC
    adc_enable(ADC0);
    Delay_ms(1);

    adc_calibration_enable(ADC0); // 使能ADC校准
}


/// @brief ADC初始化配置
void adc_config(void)
{
    adc0_config( );  // ADC2初始化配置
    /*------------------时钟配置------------------*/
    // GPIO时钟使能
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);    // 使能AF时钟
    // ADC时钟使能
    rcu_periph_clock_enable(RCU_ADC1);
    // ADC时钟4分频，最大40MHz(120MHz / 4 = 30MHz)
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);

    /*------------------ADC GPIO配置------------------*/
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_6); // U相电流
    // gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_7); // V相电流
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0); // W相电流

    adc_deinit(ADC1);   // 复位ADC0
    /*------------------ADC工作模式配置------------------*/
    adc_mode_config(ADC_MODE_FREE); // 独立模式
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);   // 使能扫描模式
    // adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);  // 使能连续转换模式

    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);// 结果转换右对齐
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 2); // 转换通道数量

    // 配置ADC通道转换顺序，采样时间(转换时间=采样时间+12.5个时钟周期=14个时钟周期=14/30MHz=0.466us)
    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_6, ADC_SAMPLETIME_7POINT5);    // U相电流
    // adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_7, ADC_SAMPLETIME_1POINT5);    // V相电流
    adc_inserted_channel_config(ADC1, 1, ADC_CHANNEL_8, ADC_SAMPLETIME_7POINT5);    // W相电流

    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_CH3);   // 定时器0通道3上升沿触发
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE); // 注入组使能

    adc_interrupt_enable(ADC1, ADC_INT_EOIC);  // 使能ADC1转换完成中断

    adc_enable(ADC1); // 使能ADC
    Delay_ms(1);

    adc_calibration_enable(ADC1); // 使能ADC校准
}

/// @brief 获取ADC未滤波采样转化值
/// @param channel ADC通道
/// @return ADC原始采样值
float adc_getRawValue(const ADC_Channel channel)
{
    switch (channel)
    {
    case _U_Value:  // U相电流
        return (((adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0) - Offset_U) * Reference_Gain)) / Gain;
        // case _V_Value:  // V相电流
        //     return (((adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_1) - Offset_V) * Reference_Gain)) / Gain;
    case _W_Value:  // W相电流
        return (((adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1) - Offset_W) * Reference_Gain)) / Gain;
    case _Power_Value:  // 电源电压
        adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL); // 软件触发ADC转换
        while (dma_flag_get(DMA0, DMA_CH0, DMA_FLAG_FTF) == RESET);  // 等待转换完成
        return *Power_ADC_Value * Reference_Gain * 6.1F; // 电压分压比为51:10
    default:
        return 0;
    }
}

/// @brief 初始化电机U、V、W相电流和电源电压偏置
/// @return 1:初始化完成 0:初始化未完成
void adc_OffsetConfig(void)
{
    uint32_t sum_U = 0, sum_W = 0, sum_Reference = 0;
    for (uint8_t i = 0; i < 200; i++)
    {
        adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL); // 软件触发ADC转换
        Delay_ms(1);//延时1ms
        sum_U += adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
        sum_W += adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1);
        sum_Reference += *Reference_ADC_Value;
    }
    Offset_U = sum_U / 200;
    Offset_W = sum_W / 200;
    Reference_Gain = REFERENCE / (sum_Reference / 200.0F);  // 内部参考电压增益
}

