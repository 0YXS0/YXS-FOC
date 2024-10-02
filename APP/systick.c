#include "systick.h"

#define _1us_count 120U //1us对应的计数值
static float Systick_time_us = 1000U; //系统时间(us)

/// @brief 系统滴答定时器配置
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        /* capture error */
        while (1);
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/// @brief 获取当前系统时间(us)
/// @return 当前系统时间(us)
float get_now_time(void)
{
    return (Systick_time_us - (SysTick->VAL / _1us_count));
}

/// @brief 系统滴答定时器中断服务函数
void SysTick_Handler(void)
{
    Systick_time_us += 1000.0F;   //时间增加1ms
}
