#include "delay.h"

/**
  * @brief          使用循环计数延迟一段时间
  * @param[in]      us:us微秒
  * @retval         none
  */
void user_delay_us(uint16_t us)
{
	for(; us>0; us--)
	{
		for(uint8_t i= 50; i>0; i--)
		{
			;
		}
	}
}
/**
  * @brief          使用user_delay_us函数延迟一段时间
  * @param[in]      ms:ms毫秒
  * @retval         none
  */
void user_delay_ms(uint16_t ms)
{
	for(; ms>0; ms--)
	{
		user_delay_us(1000);
	}
}
/**
  * @brief          使用nop函数延迟一段时间
  * @param[in]      us:us微秒
  * @retval         none
  */
//void nop_delay_us(uint16_t us)
//{
//    for(; us > 0; us--)
//    {
//        for(uint8_t i = 10; i > 0; i--)
//       {
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//            __nop();
//       }
//    }
//}
/**
  * @brief          使用nop_delay_us函数延迟一段时间
  * @param[in]      ms:ms毫秒
  * @retval         none
  */
//void nop_delay_ms(uint16_t ms)
//{
//    for(; ms > 0; ms--)
//    {
//        nop_delay_us(1000);
//    }
//}
