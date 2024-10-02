#ifndef SYSTICK_H
#define SYSTICK_H

#include "gd32f30x.h"

void systick_config(void);  //系统滴答定时器配置
float get_now_time(void);   //获取当前系统时间(us)

#endif /* SYSTICK_H */
