#ifndef  AS5600_H
#define  AS5600_H

#include "gd32f30x.h"

uint16_t AS5600_GetCount(void);    // 读取AS5600读原始角度(顺时针为正，逆时针为负)(使用I2C中断)

#endif // ! AS5600_H


