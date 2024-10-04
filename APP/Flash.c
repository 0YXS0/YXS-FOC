#include "Flash.h"
#include "gd32f30x_fmc.h"

/// @brief 擦除Flash页
/// @param startAddr 起始页开始地址
/// @param pageNum 页数
void FMCErasePage(uint32_t startAddr, uint8_t pageNum)
{
    fmc_bank0_unlock( );    //解锁bank0
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    for (char i = 0; i < pageNum; i++)
    {
        fmc_page_erase(startAddr + i * FLASH_BANK0_PAGE_SIZE);    //擦除页
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }
    fmc_bank0_lock( );  //锁定bank0
}

/// @brief 写入Flash(以4字节为单位)
/// @param startAddr 起始地址
/// @param data 数据
/// @param size 数据大小
void FMCWriteData(uint32_t startAddr, uint32_t* data, uint32_t size)
{
    fmc_bank0_unlock( );    //解锁bank0
    for (uint32_t i = 0; i < size; i++)
    {
        fmc_word_program(startAddr + i * 4, data[i]);    //写入数据
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }
    fmc_bank0_lock( );  //锁定bank0
}




