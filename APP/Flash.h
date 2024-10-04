#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

#define FLASH_BANK0_PAGE_SIZE 0x800   // bank0的页大小(2KB)
#define FLASH_MAX_ADDR 0x0803FFFF // Flash的最大地址(GD32F303CCT6的Flash最大地址为0x0803FFFF)(256KB)

#define FLASH_USER_USE_SIZE 0x800   // 用户自定义存储区域大小(2KB)
#define FLASH_USER_START_ADDR (FLASH_MAX_ADDR - FLASH_USER_USE_SIZE + 1) // 用户自定义存储区开始地址

/// Flash读取数据,直接给指针强制赋值即刻
/// 如: int* data = (int*)(0x08000000);
void FMCErasePage(uint32_t startAddr, uint8_t pageNum);    //擦除Flash页
void FMCWriteData(uint32_t startAddr, uint32_t* data, uint32_t size);    //写入Flash

#endif // !FLASH_H



