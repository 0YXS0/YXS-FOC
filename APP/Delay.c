#include "gd32f30x.h"
#include "systick.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~155344
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
  float StartTime = get_now_time( );
  while (get_now_time( ) - StartTime < xus);
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
  float StartTime = get_now_time( );
  while (get_now_time( ) - StartTime < xms * 1000);
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
  while (xs--)
  {
    Delay_ms(1000);
  }
}
