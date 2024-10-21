#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void SPI0_Config(void); // 初始化SPI0
int8_t SPI0_ReadData(uint16_t* const data); // SPI0读取数据(使用前需要先拉低片选信号)
int8_t SPI0_WriteData(const uint16_t data); // SPI0写入数据(使用前需要先拉低片选信号)

#endif // !SPI_H




