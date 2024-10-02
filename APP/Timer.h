#ifndef TIMER_H
#define TIMER_H

#include "gd32f30x.h"

#define NUM 1

void timer_config(uint16_t prescaler, uint32_t period);

#endif // !TIMER_H
