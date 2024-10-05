#include "Timer.h"
#include "gd32f30x.h"
#include "gd32f30x_timer.h"

void timer_gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
}

/// @brief 定时器初始配置
/// @param prescaler 预分频值
/// @param period 周期值
void timer_config(uint16_t prescaler, uint32_t period)
{
    timer_gpio_config( );    //配置定时器引脚

    //使能时钟
    rcu_periph_clock_enable(RCU_TIMER0);

    timer_deinit(TIMER0);   //复位定时器
    //配置定时器
    timer_parameter_struct timer_structure;
    timer_structure.alignedmode = TIMER_COUNTER_CENTER_DOWN;   //中心对齐模式,向下计数溢出触发中断
    timer_structure.clockdivision = TIMER_CKDIV_DIV1;   //时钟分频
    timer_structure.counterdirection = TIMER_COUNTER_UP;    //向上计数
    timer_structure.prescaler = prescaler - 1;    //预分频
    timer_structure.period = period - 1;          //周期
    timer_structure.repetitioncounter = 0;        //重复计数器
    timer_init(TIMER0, &timer_structure);

    // 配置死区和中止功能
    // timer_break_parameter_struct brak_structure;
    // brak_structure.breakpolarity = TIMER_BREAK_POLARITY_LOW;    //中断极性为低电平
    // brak_structure.breakstate = TIMER_BREAK_DISABLE;            //中断失能
    // brak_structure.deadtime = 12;  //死区时间(n个系统时钟周期)
    // brak_structure.ideloffstate = TIMER_IOS_STATE_DISABLE;    //空闲状态为禁用
    // brak_structure.outputautostate = TIMER_OUTAUTO_ENABLE;    //自动输出使能
    // brak_structure.protectmode = TIMER_CCHP_PROT_0;           //保护模式  
    // brak_structure.runoffstate = TIMER_ROS_STATE_DISABLE;     //运行状态为禁用
    // timer_break_config(TIMER0, &brak_structure);
    // timer_break_enable(TIMER0);   //使能中止功能

    //配置PWM参数
    timer_oc_parameter_struct pwm_structure;
    pwm_structure.ocidlestate = TIMER_OC_IDLE_STATE_LOW;    //空闲状态为低电平
    pwm_structure.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;  //空闲状态为低电平
    pwm_structure.ocpolarity = TIMER_OC_POLARITY_HIGH;      //有效电平为高电平
    pwm_structure.ocnpolarity = TIMER_OCN_POLARITY_HIGH;    //有效电平为高电平
    pwm_structure.outputstate = TIMER_CCX_ENABLE;       //输出使能
    pwm_structure.outputnstate = TIMER_CCXN_ENABLE;     //输出使能

    timer_channel_output_config(TIMER0, TIMER_CH_0, &pwm_structure);    //配置通道0
    timer_channel_output_config(TIMER0, TIMER_CH_1, &pwm_structure);    //配置通道1
    timer_channel_output_config(TIMER0, TIMER_CH_2, &pwm_structure);    //配置通道2
    timer_channel_output_config(TIMER0, TIMER_CH_3, &pwm_structure);    //配置通道3

    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);   //PWM模式1
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM1);   //PWM模式1
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM1);   //PWM模式1
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM1);   //PWM模式1
    //使能影子寄存器
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 0);   //占空比
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, 0);   //占空比
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, 0);   //占空比
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 750);   //占空比

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);

    timer_automatic_output_enable(TIMER0);    //自动输出使能

    timer_enable(TIMER0);         //使能定时器
}



