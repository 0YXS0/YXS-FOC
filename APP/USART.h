#ifndef __USART_H
#define __USART_H

#include "gd32f30x.h"

void usart_config(void);
void usart1_send_data(uint8_t* data, uint8_t dataSize);
void JustFloat_Show(uint8_t Num, ...);	//Justfloat 数据协议

#endif  //__USART_H

