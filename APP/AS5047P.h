#ifndef AS5047P_H
#define AS5047P_H

#include <stdint.h>

void AS5047P_Init(void); // AS5047P初始化
int8_t AS5047P_ReadCount(uint16_t* const data); // AS5047P读取计数值

#endif //  AS5047P_H



