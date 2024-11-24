#ifndef CAN_H
#define CAN_H

#include "gd32f30x_can.h"

void CAN0_config(uint8_t nodeID);   // 初始化CAN0
int8_t canSend(uint32_t can_periph, can_trasnmit_message_struct* m);    // 设置CANOpen数据发送函数

#endif // !CAN_H

